#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32c3_wifi.h"
#include "exec/address-spaces.h"
#include "esp32_wlan_packet.h"
#include "hw/qdev-properties.h"

#define DEBUG 0

#define ESP32C3_WIFI_RX_CTRL_HW_LEN 44
#define ESP32C3_WIFI_CSI_LEN_SHIFT 8
#define ESP32C3_WIFI_CSI_LEN_MASK (0x3ffU << ESP32C3_WIFI_CSI_LEN_SHIFT)
#define ESP32C3_WIFI_CSI_LEN 128
#define ESP32C3_WIFI_FFT_GAIN_OFFSET 22
#define ESP32C3_WIFI_AGC_GAIN_OFFSET 23
#define ESP32C3_WIFI_DEFAULT_FFT_GAIN 2
#define ESP32C3_WIFI_DEFAULT_AGC_GAIN 32
#define ESP32C3_WIFI_CSI_STABLE_NS (30LL * NANOSECONDS_PER_SECOND)
#define ESP32C3_WIFI_CSI_MOTION_NS (10LL * NANOSECONDS_PER_SECOND)
#define ESP32C3_WIFI_CSI_NOISE_PEAK 2

static int8_t esp32c3_wifi_csi_clamp(double value)
{
    if (value > 127.0) {
        return 127;
    }
    if (value < -127.0) {
        return -127;
    }
    return (int8_t)lrint(value);
}

static int esp32c3_wifi_csi_white_noise(void)
{
    return rand() % (2 * ESP32C3_WIFI_CSI_NOISE_PEAK + 1) -
           ESP32C3_WIFI_CSI_NOISE_PEAK;
}

static void esp32c3_wifi_generate_csi(uint8_t *buf, size_t len,
                                      int64_t now_ns)
{
    const int64_t cycle_ns = ESP32C3_WIFI_CSI_STABLE_NS +
                             ESP32C3_WIFI_CSI_MOTION_NS;
    const int64_t cycle_pos = now_ns % cycle_ns;
    const bool motion = cycle_pos >= ESP32C3_WIFI_CSI_STABLE_NS;
    double phase1 = 0.35;
    double phase2 = -0.90;
    double path1_gain = 26.0;
    double path2_gain = 15.0;
    double direct_gain = 48.0;

    if (motion) {
        double t = (cycle_pos - ESP32C3_WIFI_CSI_STABLE_NS) /
                   (double)NANOSECONDS_PER_SECOND;

        /*
         * Human motion changes reflected-path phase and amplitude quickly in
         * time, but each instantaneous frequency response remains smooth.
         */
        phase1 += 5.5 * sin(2.0 * G_PI * 1.7 * t);
        phase2 += 7.0 * cos(2.0 * G_PI * 2.3 * t);
        path1_gain = 34.0 + 10.0 * sin(2.0 * G_PI * 1.1 * t);
        path2_gain = 24.0 + 8.0 * cos(2.0 * G_PI * 1.9 * t);
        direct_gain = 42.0 + 4.0 * sin(2.0 * G_PI * 0.7 * t);
    }

    /* ESP32 CSI is interleaved imaginary/real data for each subcarrier. */
    for (size_t i = 0; i + 1 < len; i += 2) {
        double subcarrier = (double)(i / 2) - (double)(len / 4);
        double path1_phase = phase1 + subcarrier * 0.055;
        double path2_phase = phase2 - subcarrier * 0.130;
        double real = direct_gain + path1_gain * cos(path1_phase) +
                      path2_gain * cos(path2_phase) +
                      esp32c3_wifi_csi_white_noise();
        double imag = path1_gain * sin(path1_phase) +
                      path2_gain * sin(path2_phase) +
                      esp32c3_wifi_csi_white_noise();

        buf[i] = (uint8_t)esp32c3_wifi_csi_clamp(imag);
        buf[i + 1] = (uint8_t)esp32c3_wifi_csi_clamp(real);
    }
}

static uint64_t esp32C3_wifi_read(void *opaque, hwaddr addr, unsigned int size)
{
    
    Esp32WifiState *s = ESP32_WIFI(opaque);
    uint32_t r = s->mem[addr/4];
    
    switch(addr) {
        case A_C3_WIFI_DMA_IN_STATUS:
            r=0;
            break;
        case A_C3_WIFI_DMA_INT_STATUS:
        case A_C3_WIFI_DMA_INT_CLR:
            r=s->raw_interrupt;
            break;
        case A_C3_WIFI_STATUS:
        case A_C3_WIFI_DMA_OUT_STATUS:
            r=1;
            break;           
    }

    if(DEBUG) printf("esp32C3_wifi_read  0x%04lx= 0x%08x\n",(unsigned long) addr,r);

    return r;
}
static void set_interrupt(Esp32WifiState *s,int e) {
    s->raw_interrupt |= e;
    qemu_set_irq(s->irq, 1);
}

void Esp32_WLAN_frame_delivered(Esp32WifiState *s){
    s->raw_interrupt |= 0x80;
    qemu_set_irq(s->irq, 1);
}

static void esp32C3_wifi_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned int size) {
    Esp32WifiState *s = ESP32_WIFI(opaque);
    if(DEBUG) printf("esp32C3_wifi_write 0x%04lx= 0x%08lx\n",(unsigned long) addr, (unsigned long) value);

    switch (addr) {
        case A_C3_WIFI_DMA_INLINK:
            s->dma_inlink_address = value;
            break;
        case A_C3_WIFI_DMA_INT_CLR:
            s->raw_interrupt &= ~value;
            if(s->raw_interrupt == 0)
                qemu_set_irq(s->irq, 0);
            break;
        case A_C3_WIFI_DMA_OUTLINK:
            if (value & 0xc0000000) {                        
                // do a DMA transfer to the hardware from esp32 memory
                mac80211_frame frame;
                dma_list_item item;
                unsigned memaddr = (0x3fc00000 | (value & 0xfffff));
                address_space_read(&address_space_memory, memaddr,
                            MEMTXATTRS_UNSPECIFIED, &item, 12);
                address_space_read(&address_space_memory, item.address,
                            MEMTXATTRS_UNSPECIFIED, &frame, item.length);
                // frame from esp32 to ap
                frame.frame_length=item.length;
                frame.next_frame=0;
                Esp32_WLAN_handle_frame(s, &frame);
                set_interrupt(s,0x80);
            }
    }
    s->mem[addr/4]=value;
}

static int match_mac_address(uint8_t *a1,uint8_t *a2) {
    if(!memcmp(a1,a2,6)) return 1;
    if(!memcmp(a1,BROADCAST,6)) return 1;
    return 0;
}
// frame from ap to esp32
void Esp32_sendFrame(Esp32WifiState *s, mac80211_frame *frame,int length, int signal_strength) {
    if(s->dma_inlink_address==0) return;

    bool is_mgmt = frame->frame_control.type == IEEE80211_TYPE_MGT;
    bool is_beacon = is_mgmt && frame->frame_control.sub_type ==
                     IEEE80211_TYPE_MGT_SUBTYPE_BEACON;
    bool is_espnow = is_mgmt && frame->frame_control.sub_type ==
                     IEEE80211_TYPE_MGT_SUBTYPE_ACTION;
    bool has_csi = is_beacon || is_espnow;
    int csi_len = has_csi ? ESP32C3_WIFI_CSI_LEN : 0;
    int total_len = sizeof(wifi_pkt_rx_ctrl_c3_t) + csi_len + length;
    uint8_t *header=malloc(total_len);
    memset(header,0,total_len);
    wifi_pkt_rx_ctrl_c3_t *pkt=(wifi_pkt_rx_ctrl_c3_t *)header;

    *pkt=(wifi_pkt_rx_ctrl_c3_t){
        .rssi=(signal_strength+(rand()%10) -  60),
        .rate=11,
        .sig_len=length,
        .sig_len_copy=length,
        .legacy_length=length,
        .noise_floor=-97,
        .channel=esp32_wifi_channel,
        .timestamp=qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)/1000,
    };

    /*
     * These values occupy reserved bytes in the public ESP32-C3 RX-control
     * definition, but the CSI gain-control component consumes them directly:
     * byte 22 is signed FFT gain and byte 23 is unsigned AGC gain.
     */
    header[ESP32C3_WIFI_FFT_GAIN_OFFSET] =
        (uint8_t)(int8_t)ESP32C3_WIFI_DEFAULT_FFT_GAIN;
    header[ESP32C3_WIFI_AGC_GAIN_OFFSET] =
        ESP32C3_WIFI_DEFAULT_AGC_GAIN;

    // These 4 bits are set if the mac addresses previously stored at 0x40 and 0x48
    // match the destination or bssid addresses in the frame
    if(match_mac_address(frame->destination_address,(uint8_t *)s->mem+0x40))
        pkt->damatch0=1;
    if(match_mac_address(frame->destination_address,(uint8_t *)s->mem+0x48))
        pkt->damatch1=1;
    if(match_mac_address(frame->bssid_address,(uint8_t *)s->mem+0x40))
        pkt->bssidmatch0=1;
    if(match_mac_address(frame->bssid_address,(uint8_t *)s->mem+0x48))
        pkt->bssidmatch1=1;

    if (has_csi) {
        uint32_t *csi_info = (uint32_t *)(header + 28);

        /*
         * The ESP32-C3 hardware RX buffer does not append CSI after the
         * 802.11 frame.  Its private layout is:
         *
         *   RX control[0..43], CSI, RX control[44..47], 802.11 frame
         *
         * Bits 8..17 of the word at RX-control offset 28 contain the CSI
         * length.  IDF's wdev_csi_len_align() reads that private field and
         * removes the CSI block while converting the hardware buffer into
         * the public RX-control-plus-frame representation.
         */
        *csi_info = (*csi_info & ~ESP32C3_WIFI_CSI_LEN_MASK) |
                    ((uint32_t)csi_len << ESP32C3_WIFI_CSI_LEN_SHIFT);
        memmove(header + ESP32C3_WIFI_RX_CTRL_HW_LEN + csi_len,
                header + ESP32C3_WIFI_RX_CTRL_HW_LEN,
                sizeof(wifi_pkt_rx_ctrl_c3_t) -
                ESP32C3_WIFI_RX_CTRL_HW_LEN);

        esp32c3_wifi_generate_csi(header + ESP32C3_WIFI_RX_CTRL_HW_LEN,
                                  csi_len,
                                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }

    memcpy(header + sizeof(wifi_pkt_rx_ctrl_c3_t) + csi_len,
           frame, length);

    // do a DMA transfer from the hardware to esp32 memory
    dma_list_item item;
    address_space_read(&address_space_memory, s->dma_inlink_address, MEMTXATTRS_UNSPECIFIED, &item, 12);
    address_space_write(&address_space_memory, item.address, MEMTXATTRS_UNSPECIFIED, header, total_len);
    item.length=total_len;
    item.eof=1;
    address_space_write(&address_space_memory, s->dma_inlink_address, MEMTXATTRS_UNSPECIFIED,&item,4);
    s->dma_inlink_address=item.next;
    set_interrupt(s, 0x1004024);
    free(header);
}

static const MemoryRegionOps esp32C3_wifi_ops = {
    .read =  esp32C3_wifi_read,
    .write = esp32C3_wifi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32c3_wifi_reset_enter(Object *obj, ResetType type)
{
    Esp32WifiState *s = ESP32_WIFI(obj);

    s->dma_inlink_address=0;
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_reset_ap(s);
}

static void esp32C3_wifi_realize(DeviceState *dev, Error **errp)
{
    Esp32WifiState *s = ESP32_WIFI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    s->dma_inlink_address=0;

    memory_region_init_io(&s->iomem, OBJECT(dev), &esp32C3_wifi_ops, s,
                          TYPE_ESP32_WIFI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    memset(s->mem,0,sizeof(s->mem));
    Esp32_WLAN_setup_ap(dev, s);
}
static Property esp32C3_wifi_properties[] = {
    DEFINE_NIC_PROPERTIES(Esp32WifiState, conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32C3_wifi_class_init(ObjectClass *klass, void *data)
{
	ResettablePhases rp;
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = esp32C3_wifi_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->desc = "Esp32C3 WiFi";
    device_class_set_props(dc, esp32C3_wifi_properties);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    resettable_class_set_parent_phases(rc, esp32c3_wifi_reset_enter, NULL, NULL, &rp);
}


static const TypeInfo esp32C3_wifi_info = {
    .name = TYPE_ESP32_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32WifiState),
    .class_init    = esp32C3_wifi_class_init,
};

static void esp32C3_wifi_register_types(void)
{
    type_register_static(&esp32C3_wifi_info);
}

type_init(esp32C3_wifi_register_types)

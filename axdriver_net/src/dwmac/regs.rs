pub mod phy {
    // https://www.lcsc.com/datasheet/lcsc_datasheet_2410121612_Motorcomm-YT8531C_C5357044.pdf
    use bitflags::bitflags;

    bitflags! {
        // 0xA001
        pub struct YT8531CChipCfg: u16 {
            const SW_RST_N_MODE = 1 << 15;
            const EN_SDS_SLEEP = 1 << 14;
            const MBIST_MODE = 1 << 13;
            const MBIST_CLK_SEL = 1 << 12;
            const IDDQ_MODE = 1 << 11;
            const RESERVED_10 = 1 << 10;
            const EN_GATE_RX_CLK_RGMII = 1 << 9;
            const RXC_DLY_EN = 1 << 8;
            const RESERVED_7 = 1 << 7;
            const EN_LDO = 1 << 6;
            const CFG_LDO_1_8V = 1 << 5;
            const CFG_LDO_2_5V = 1 << 4;
            const RESERVED_3 = 1 << 3;
            const RESERVED_2 = 1 << 2;
            const RESERVED_1 = 1 << 1;
            const RESERVED_0 = 1 << 0;
            const CFG_LDO_3_3V = 0;
        }
    }

    // Bit 15 (YT8521_CCR_SW_RST): 1 = Software Reset. <br> Bits 5:4 (YT8531_RGMII_LDO_VOL_MASK): 0b10 = YT8531_LDO_VOL_1V8 (1.8V RGMII LDO voltage). <br> Bits 2:0 (YT8521_CCR_MODE_SEL_MASK): 0b000 = YT8521_CCR_MODE_UTP_TO_RGMII (UTP to RGMII mode).
    /// 0x8020
    pub const CHIP_CFG_DEFAULT: u16 =
        YT8531CChipCfg::SW_RST_N_MODE.bits() | YT8531CChipCfg::CFG_LDO_1_8V.bits();

    pub const PAD_DRIVER_STRENGTH_CFG: u16 = 0xA010;
    bitflags! {
        // Table 28. Pad Drive Strength Cfg (EXT_0xA010)
        // Bit Symbol Access Default Description
        // 15:13Rgmii_sw_dr_rxcRW0x3None
        // 12 Rgmii_sw_dr[2]RW0x0None
        // 11 Int_od_enRW0x1None
        // 10 Int_act_hiRW0x0None
        // 9:8 Dr_sync_eRW0x3Drive strenght of sync e pad. 2'b11: strongest; 2'b00: weakest
        // 7:6Dr_mdioRW0x3Drive strenght of mdio pad. 2'b11: strongest; 2'b00: weakest
        // 5:4Rgmii_sw_dr[1:0]RW POS0x3Drive strenght of rx rgmii pad. 2'b11: strongest; 2'b00: weakest, depend on rgmii IO voltage level
        // 3:2Dr_ptp_ioRW0x3Drive strenght of rx ptp io pad. 2'b11: strongest; 2'b00: weakest
        // 1:0Dr_ledRW0x3Drive strenght of led io pad. 2'b11: strongest; 2'b00: weakest

        pub struct PadDriverStrengthCfg: u16 {
            const RGMII_SW_DR_RXC_2 = 1 << 15;
            const RGMII_SW_DR_RXC_1 = 1 << 14;
            const RGMII_SW_DR_RXC_0 = 1 << 13;
            const RGMII_SW_DR_2 = 1 << 12;
            const INT_OD_EN = 1 << 11;
            const INT_ACT_HI = 1 << 10;
            const DR_SYNC_E_1 = 1 << 9;
            const DR_SYNC_E_0 = 1 << 8;
            const DR_MDIO_1 = 1 << 7;
            const DR_MDIO_0 = 1 << 6;
            const RGMII_SW_DR_1 = 1 << 5;
            const RGMII_SW_DR_0 = 1 << 4;
            const DR_PTPIO_1 = 1 << 3;
            const DR_PTPIO_0 = 1 << 2;
            const DR_LED_1 = 1 << 1;
            const DR_LED_0 = 1 << 0;
        }
    }

    /// 0xcbff = 0b1100101111111111
    pub const PAD_DRIVER_STRENGTH_CFG_DEFAULT: u16 = PadDriverStrengthCfg::RGMII_SW_DR_RXC_2.bits()
        | PadDriverStrengthCfg::RGMII_SW_DR_RXC_1.bits()
        | PadDriverStrengthCfg::INT_OD_EN.bits()
        | PadDriverStrengthCfg::DR_SYNC_E_1.bits()
        | PadDriverStrengthCfg::DR_SYNC_E_0.bits()
        | PadDriverStrengthCfg::DR_MDIO_1.bits()
        | PadDriverStrengthCfg::DR_MDIO_0.bits()
        | PadDriverStrengthCfg::RGMII_SW_DR_1.bits()
        | PadDriverStrengthCfg::RGMII_SW_DR_0.bits()
        | PadDriverStrengthCfg::DR_PTPIO_1.bits()
        | PadDriverStrengthCfg::DR_PTPIO_0.bits()
        | PadDriverStrengthCfg::DR_LED_1.bits()
        | PadDriverStrengthCfg::DR_LED_0.bits();

    pub const RGMII_CONFIG_1: u16 = 0xA003;
    bitflags! {
        // Table 15. RGMII_Config1 (EXT_0xA003)
        // Bit Symbol Access Default Description
        // 15 ReservedRW0x0Reserved
        // 14 Tx_clk_selRW0x00: use original RGMII TX_CLK to drive the RGMII TX_CLK delay train;1: use inverted RGMII TX_CLK to drive the RGMII TX_CLK delay train. Used for debug
        // 13:10Rx_delay_selRW0x0RGMII RX_CLK delay train configuration, about 150ps per step
        // 9 En_rgmii_fd_crsRW0x0See EXT 0xA003 bit[8].
        // 8 En_rgmii_crsRW0x00: to not encode GMII/MII CRS into RGMII OOB; 1: to encode GMII/MII CRS into RGMII OOB when it's half duplex mode or EXT 0xA003 bit[9] is 1.
        // 7:4Tx_delay_sel_feRW0xfRGMII TX_CLK delay train configuration when speed is 100Mbps or 10Mbps, it's 150ps per step typically.
        // 3:0Tx_delay_selRW0x1 RGMII TX_CLK delay train configuration when speed is 1000Mbps, it's150ps per step typically.

        pub struct RGMIIConfig1: u16 {
            const RESERVED_15 = 1 << 15;
            const TX_CLK_SEL = 1 << 14;
            const RX_DELAY_SEL_3 = 1 << 13;
            const RX_DELAY_SEL_2 = 1 << 12;
            const RX_DELAY_SEL_1 = 1 << 11;
            const RX_DELAY_SEL_0 = 1 << 10;
            const EN_RGMII_FD_CRS = 1 << 9;
            const EN_RGMII_CRS = 1 << 8;
            const TX_DELAY_SEL_FE_3 = 1 << 7;
            const TX_DELAY_SEL_FE_2 = 1 << 6;
            const TX_DELAY_SEL_FE_1 = 1 << 5;
            const TX_DELAY_SEL_FE_0 = 1 << 4;
            const TX_DELAY_SEL_3 = 1 << 3;
            const TX_DELAY_SEL_2 = 1 << 2;
            const TX_DELAY_SEL_1 = 1 << 1;
            const TX_DELAY_SEL_0 = 1 << 0;
        }
    }

    /// 0x0850 = 0b0000100001010000
    pub const RGMII_CONFIG_1_DEFAULT: u16 = RGMIIConfig1::RX_DELAY_SEL_1.bits()
        | RGMIIConfig1::TX_DELAY_SEL_FE_2.bits()
        | RGMIIConfig1::TX_DELAY_SEL_FE_0.bits();
}

/// Register offsets
pub mod mac {
    use bitflags::bitflags;

    pub const CONFIG: usize = 0x0000;
    pub const FRAME_FILTER: usize = 0x0008;
    pub const GMII_ADDRESS: usize = 0x0200;
    pub const GMII_DATA: usize = 0x0204;

    /// Extended Register's Address Offset Register
    pub const YTPHY_PAGE_SELECT: usize = GMII_ADDRESS + 0x001E;

    /// Extended Register's Data Register
    pub const YTPHY_PAGE_DATA: usize = GMII_ADDRESS + 0x001F;

    pub const VERSION: usize = 0x0110;
    pub const ADDRESS0_HIGH: usize = 0x0300;
    pub const ADDRESS0_LOW: usize = 0x0304;
    pub const INTERRUPT_STATUS: usize = 0x00b0;
    pub const INTERRUPT_ENABLE: usize = 0x00b4;

    /*  MAC Interrupt bitmap*/
    // #define GMAC_INT_RGMII		BIT(0)
    // #define GMAC_INT_PCS_LINK		BIT(1)
    // #define GMAC_INT_PCS_ANE		BIT(2)
    // #define GMAC_INT_PCS_PHYIS		BIT(3)
    // #define GMAC_INT_PMT_EN			BIT(4)
    // #define GMAC_INT_LPI_EN			BIT(5)
    // #define GMAC_INT_TSIE			BIT(12)

    // #define	GMAC_PCS_IRQ_DEFAULT	(GMAC_INT_RGSMIIS | GMAC_INT_PCS_LINK |	\
    // 				 GMAC_INT_PCS_ANE)

    // #define	GMAC_INT_DEFAULT_ENABLE	(GMAC_INT_PMT_EN | GMAC_INT_LPI_EN | \
    //     GMAC_INT_TSIE)
    bitflags! {
        pub struct MacInterruptEnable: u32 {
            const RGMII = 1 << 0;
            const PCS_LINK = 1 << 1;
            const PCS_ANE = 1 << 2;
            const PCS_PHYIS = 1 << 3;
            const PMT_EN = 1 << 4;
            const LPI_EN = 1 << 5;
            const TSIE = 1 << 12;
        }
    }

    pub const PCS_IRQ_DEFAULT: u32 = MacInterruptEnable::PMT_EN.bits()
        | MacInterruptEnable::LPI_EN.bits()
        | MacInterruptEnable::TSIE.bits();

    pub const INT_DEFAULT_ENABLE: u32 =
        MacInterruptEnable::PCS_LINK.bits() | MacInterruptEnable::PCS_ANE.bits();

    // /* MAC config */
    // #define GMAC_CONFIG_RE			BIT(0)
    // Bit(s)	Mnemonic (Typical)	Value (from 0x8072203)	Binary Value	Description and Implication
    // 31	SWR	1	1	Software Reset: 1 = Asserts software reset for the MAC. All MAC internal logic and registers are reset. This bit is typically self-clearing or must be cleared by a subsequent software write to 0.
    // 30	BE	0	0	Byte Endianness (for Data Buffers): 0 = Little Endian mode for data buffer descriptors and data. (Assumed default, may vary by IP configuration)
    // 29	DOOD	0	0	Disable Out Of Order Data Transfers: 0 = Out of order data transfers by DMA are enabled (if supported by IP). (Assumed, may vary)
    // 28	DCRCC	0	0	Disable CRC Checking for Received Frames: 0 = CRC check is performed on received frames by the MAC. (Assumed, may vary)
    // 27	SPEN	0	0	Slow Protocol Enable: 0 = Detection of IEEE 802.3ax slow protocol frames is disabled.
    // 26	USP	0	0	Unicast Slow Protocol Packet Detect: 0 = Detection of unicast slow protocol packets is disabled.
    // 25:24	PRELEN	00	00	Preamble Length for Transmit: 00 = 7 bytes of preamble will be transmitted.
    // 23	ARPEN	0	0	ARP Offload Enable: 0 = Hardware ARP response offloading is disabled.
    // 22:20	SARC	001	001	Source Address Insertion/Replacement Control: 001 = Insert MACSourceAddr (from MAC Address Registers) into transmitted frames.
    // 19	TWOKPEN	1	1	IEEE 802.3as 2K Packet Enable: 1 = Enabled. Allows transmission and reception of frames up to 2000 bytes (if JE is also appropriately set or if this overrides JE for this specific size).
    // 18	CST	1	1	CRC Stripping for Type Frames: 1 = CRC field is stripped from received Type frames (e.g., VLAN-tagged frames with EtherType not matching length).
    // 17	TC	1	1	Transmit Configuration in RGMII/SGMII: 1 = Enabled. For RGMII, this often relates to enabling MAC-generated TX clock delay, crucial for rgmii-id mode.
    // 16	WD	1	1	Watchdog Disable: 1 = MAC transmitter watchdog timer is disabled. This timer normally prevents excessively long transmissions.
    // 15	JD	1	1	Jabber Disable: 1 = MAC transmitter jabber timer is disabled. This timer normally prevents continuous transmission.
    // 14	JE	0	0	Jumbo Frame Enable: 0 = Jumbo frame transmission/reception is disabled. Maximum frame size is standard (1518/1522 bytes, or 2000 if TWOKPEN=1).
    // 13	PS	0	0	Port Select (Interface Type): 0 = Indicates GMII/MII for 1Gbps/100Mbps/10Mbps operations. This setting is used for RGMII.
    // 12	FES	0	0	Speed Select: 0 (when PS=0). In GMII/RGMII mode, this typically selects 1000 Mbps operation.
    // 11	DO	0	0	Disable Receive Own: 0 = MAC receives frames that it transmits (normal in half-duplex, or for loopback tests).
    // 10	LM	0	0	Loopback Mode: 0 = Loopback mode is disabled. Normal operation.
    // 9	DM	1	1	Duplex Mode: 1 = Full-Duplex mode is selected.
    // 8	IPC	0	0	IPv4 Checksum Offload (Rx): 0 = Checksum offload for received IPv4 frames is disabled. The MAC will not verify IPv4 header checksums.
    // 7	DR	0	0	Disable Retry Transmission: 0 = Retry transmission on collision is enabled (primarily relevant for half-duplex mode, but DM is set to Full-Duplex).
    // 6	LUD	0	0	Link Up/Down in RGMII/SGMII: 0 (Assumed default; behavior can depend on specific DWMAC version and RGMII/SGMII mode configuration).
    // 5	ACS	0	0	Automatic Pad/CRC Stripping: 0 = Padding and CRC are not stripped from received frames by the MAC. The full frame is passed to higher layers.
    // 4:3	BL	00	00	Back-Off Limit: 00 = Sets the back-off limit to k
    // coll
    // ​
    //   (number of collisions) between 0 and 10 (standard for IEEE 802.3). Relevant for half-duplex.
    // 2	DC	0	0	Deferral Check: 0 = Deferral check in the transmitter is disabled (primarily relevant for half-duplex mode).
    // 1	TE	1	1	Transmitter Enable: 1 = The MAC transmitter is enabled (will become active after SWR is cleared).
    // 0	RE	1	1	Receiver Enable: 1 = The MAC receiver is enabled (will become active after SWR is cleared).
    bitflags! {
        pub struct MacConfig: u32 {
            const SWR = 1 << 31;
            /// Source Address Insertion/Replacement Control
            const SARC = 1 << 20;
            const TWOKPEN = 1 << 19;
            const CST = 1 << 18;
            const TC = 1 << 17;
            const WD = 1 << 16;
            const JD = 1 << 15;
            const JE = 1 << 14;
            const PS = 1 << 13;
            const FES = 1 << 12;
            const DO = 1 << 11;
            const LM = 1 << 10;
            const DM = 1 << 9;
            const IPC = 1 << 8;
            const DR = 1 << 7;
            const LUD = 1 << 6;
            const ACS = 1 << 5;
            const BL = 1 << 4;
            const DC = 1 << 3;
            const TE = 1 << 1;
            const RE = 1 << 0;
        }
    }

    pub const CONFIG_DEFAULT: u32 = MacConfig::SWR.bits()
        | MacConfig::SARC.bits()
        | MacConfig::TWOKPEN.bits()
        | MacConfig::CST.bits()
        | MacConfig::TC.bits()
        | MacConfig::WD.bits()
        | MacConfig::JD.bits()
        | MacConfig::DM.bits()
        | MacConfig::TE.bits()
        | MacConfig::RE.bits();

    /// 0x78200 = 0b1111000001000000000
    pub const CONFIG_2: u32 = MacConfig::CST.bits()
        | MacConfig::TC.bits()
        | MacConfig::WD.bits()
        | MacConfig::JD.bits()
        | MacConfig::DM.bits();

    bitflags! {
        pub struct PacketFilter: u32 {
            const PR = 1 << 0;
            const HMC = 1 << 2;
            const PM = 1 << 4;
            const PCF = 1 << 7;
            const HPF = 1 << 10;
            const VTFE = 1 << 16;
            const IPFE = 1 << 20;
            const RA = 1 << 31;
        }
    }
}

/// DMA registers
pub mod dma {
    use bitflags::bitflags;

    pub const BUS_MODE: usize = 0x1000;
    pub const SYS_BUS_MODE: usize = 0x1004;
    pub const INTR_STATUS: usize = 0x1008;
    pub const CHAN_BASE_ADDR: usize = 0x1100;
    pub const CHAN_TX_CTRL: usize = CHAN_BASE_ADDR + 0x04;
    pub const CHAN_RX_CTRL: usize = CHAN_BASE_ADDR + 0x08;
    pub const CHAN_TX_BASE_ADDR_HI: usize = CHAN_TX_BASE_ADDR + 0x10;
    pub const CHAN_TX_BASE_ADDR: usize = CHAN_BASE_ADDR + 0x14;
    pub const CHAN_RX_BASE_ADDR_HI: usize = CHAN_RX_BASE_ADDR + 0x18;
    pub const CHAN_RX_BASE_ADDR: usize = CHAN_BASE_ADDR + 0x1c;
    pub const CHAN_TX_END_ADDR: usize = CHAN_BASE_ADDR + 0x20;
    pub const CHAN_RX_END_ADDR: usize = CHAN_BASE_ADDR + 0x28;
    pub const CHAN_TX_RING_LEN: usize = CHAN_BASE_ADDR + 0x2c;
    pub const CHAN_RX_RING_LEN: usize = CHAN_BASE_ADDR + 0x30;

    pub const CHAN_INTR_ENABLE: usize = CHAN_BASE_ADDR + 0x34;

    pub const TX_POLL_DEMAND: usize = 0x1004;
    pub const RX_POLL_DEMAND: usize = 0x1008;
    pub const STATUS: usize = 0x1014;
    pub const OPERATION_MODE: usize = 0x1018;

    // DMA_BUS_MODE bits
    pub const DMA_RESET: u32 = 1 << 0;

    // DMA_OPERATION_MODE bits
    pub const DMA_START_TX: u32 = 1 << 0;
    pub const DMA_TX_PBL_8: u32 = 8 << 16;
    pub const DMA_START_RX: u32 = 1 << 0;
    pub const DMA_RX_PBL_8: u32 = 8 << 16;

    // Descriptor bits
    pub const DESC_OWN: u32 = 1 << 31;
    pub const DESC_FIRST: u32 = 1 << 28;
    pub const DESC_LAST: u32 = 1 << 29;

    // TX描述符状态位（TDES0）
    pub const TX_FIRST_SEGMENT: u32 = 1 << 28; // 第一段[9]
    pub const TX_LAST_SEGMENT: u32 = 1 << 29; // 最后一段[9]
    pub const TX_INTERRUPT_COMPLETE: u32 = 1 << 30; // 完成中断[9]
    pub const TX_ERROR_SUMMARY: u32 = 1 << 15; // 错误总结位

    // RX描述符状态位（RDES0）
    pub const RX_FIRST_DESCRIPTOR: u32 = 1 << 9; // 第一个描述符[23]
    pub const RX_LAST_DESCRIPTOR: u32 = 1 << 8; // 最后一个描述符[23]
    pub const RX_ERROR_SUMMARY: u32 = 1 << 15; // 错误总结位[23]
    pub const RX_DESCRIPTOR_ERROR: u32 = 1 << 14; // 描述符错误[23]
    pub const RX_FRAME_LENGTH_MASK: u32 = 0x3FFF0000; // 帧长度字段[23]
    pub const RX_FRAME_LENGTH_SHIFT: u32 = 16; // 帧长度位移[23]

    pub const GMAC_Q0_TX_FLOW_CTRL: usize = 0x0070;
    pub const GMAC_Q0_TX_FLOW_CTRL_TFE: u32 = 0xffff_0002;

    pub const MTL_CHAN_RX_OP_MODE: usize = 0x0d00 + 0x30;

    pub const DMA_BUS_MODE_INTM_MODE1: u32 = 0b1 << 16;

    bitflags! {
        pub struct DmaSysBusMode: u32 {
            const MB = 1 << 14;
            const AXI_1KBBE = 1 << 13;
            const AAL = 1 << 12;
            const EAME = 1 << 11;
            const AXI_BLEN256 = 1 << 7;
            const AXI_BLEN128 = 1 << 6;
            const AXI_BLEN64 = 1 << 5;
            const AXI_BLEN32 = 1 << 4;
            const AXI_BLEN16 = 1 << 3;
            const AXI_BLEN8 = 1 << 2;
            const AXI_BLEN4 = 1 << 1;
            const FB = 1 << 0;
        }
    }

    bitflags! {

    // #define DMA_CHAN_INTR_NORMAL_4_10	(DMA_CHAN_INTR_ENA_NIE_4_10 | \
    //     DMA_CHAN_INTR_ENA_RIE | \
    //     DMA_CHAN_INTR_ENA_TIE)

    // #define DMA_CHAN_INTR_ABNORMAL_4_10	(DMA_CHAN_INTR_ENA_AIE_4_10 | \
    //     DMA_CHAN_INTR_ENA_FBE)

    // #define DMA_CHAN_INTR_DEFAULT_MASK_4_10	(DMA_CHAN_INTR_NORMAL_4_10 | \
    //     DMA_CHAN_INTR_ABNORMAL_4_10)
        pub struct InterruptMask: u32 {
            const TIE = 1 << 0;
            const TSE = 1 << 1;
            const TBUE = 1 << 2;
            const RIE = 1 << 6;
            const RBUE = 1 << 7;
            const RSE = 1 << 8;
            const RWE = 1 << 9;
            const ETE = 1 << 10;
            const ERE = 1 << 11;
            const FBE = 1 << 12;
            const CDE = 1 << 13;
            const AIE_4_10 = 1 << 14;
            const NIE_4_10 = 1 << 15;
            const AIE = 1 << 15;
            const NIE = 1 << 16;
            const NORMAL_4_10 = Self::NIE_4_10.bits()
                              | Self::RIE.bits()
                              | Self::TIE.bits();
            const ABNORMAL_4_10 = Self::AIE_4_10.bits()
                                | Self::FBE.bits();
            const DEFAULT_MASK_4_10 = Self::NORMAL_4_10.bits()
                                    | Self::ABNORMAL_4_10.bits();
        }
    }
}

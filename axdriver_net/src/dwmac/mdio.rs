/// dump from linux
/// phytool read eth1/1/0xa000
/// 0x1140
/// phytool read eth1/1/0xa001
/// 0x7960
/// phytool read eth1/1/0xa002
/// 0x4f51
/// phytool read eth1/1/0xa003
/// 0xe91b
/// phytool read eth1/1/0xa004
/// 0x1de1
/// phytool read eth1/1/0xa005
/// 0xc5e1
/// phytool read eth1/1/0xa006
/// 0x000d
/// phytool read eth1/1/0xa007
/// 0x2801
/// phytool read eth1/1/0xa008
/// 0x0000
/// phytool read eth1/1/0xa009
/// 0x0300
/// phytool read eth1/1/0xa00a
/// 0x3800
/// phytool read eth1/1/0xa00b
/// 0x0000
/// phytool read eth1/1/0xa00c
/// 0x0000
/// phytool read eth1/1/0xa00d
/// 0x4007
/// phytool read eth1/1/0xa00e
/// 0x0000
/// phytool read eth1/1/0xa00f
/// 0x2000

// EQOS MAC MDIO寄存器偏移
pub const MAC_MDIO_ADDRESS_OFFSET: usize = 0x200;
pub const MAC_MDIO_DATA_OFFSET: usize = 0x204;

// MAC_MDIO_Address寄存器位字段定义
pub const MAC_MDIO_ADDRESS_GB: u32 = 1 << 0; // Go/Busy位
pub const MAC_MDIO_ADDRESS_C45E: u32 = 1 << 1; // Clause 45 Enable
pub const MAC_MDIO_ADDRESS_GOC_SHIFT: u32 = 2; // Go/Busy Operation Code
pub const MAC_MDIO_ADDRESS_GOC_READ: u32 = 0x3; // 读操作
pub const MAC_MDIO_ADDRESS_GOC_WRITE: u32 = 0x1; // 写操作
pub const MAC_MDIO_ADDRESS_SKAP: u32 = 1 << 4; // Skip Address Packet
pub const MAC_MDIO_ADDRESS_CR_SHIFT: u32 = 8; // Clock Range
pub const MAC_MDIO_ADDRESS_CR: u32 = 0x4; // 时钟范围设置
pub const MAC_MDIO_ADDRESS_PA_SHIFT: u32 = 21; // PHY Address
pub const MAC_MDIO_ADDRESS_RDA_SHIFT: u32 = 16; // Register/Device Address

// YT8531C标准寄存器
pub const YT8531C_BMCR: u16 = 0x00; // Basic Mode Control Register
pub const YT8531C_BMSR: u16 = 0x01; // Basic Mode Status Register
pub const YT8531C_PHYID1: u16 = 0x02; // PHY Identifier 1
pub const YT8531C_PHYID2: u16 = 0x03; // PHY Identifier 2

// YT8531C扩展寄存器访问
pub const YT8531C_CLOCK_GATING_REG: u16 = 0xc;
pub const REG_DEBUG_ADDR_OFFSET: u16 = 0x1e; // 扩展寄存器地址寄存器
pub const REG_DEBUG_DATA: u16 = 0x1f; // 扩展寄存器数据寄存器

// YT8531C扩展寄存器地址
pub const YT8531C_EXT_SLEEP_CONTROL1: u16 = 0x0027;
pub const YT8531C_EXT_CLK_OUTPUT: u16 = 0xa001;
pub const YT8531C_EXT_CHIP_CONFIG: u16 = 0xa001;
pub const YT8531C_EXT_RGMII_CONFIG1: u16 = 0xa003;
pub const YT8531C_EXT_CLK_TX_INVERT: u16 = 0xa010;
pub const YT8531C_EXT_SYNCE_CFG: u16 = 0xa012;

// 延迟配置位定义
const YT8531_CCR_RXC_DLY_EN: u16 = 1 << 8; // RX时钟延迟使能
const YT8531_RC1R_RX_DELAY_MASK: u16 = 0xF000;
const YT8531_RC1R_GE_TX_DELAY_MASK: u16 = 0x0F00;
const YT8531_CGR_RX_CLK_EN: u16 = 1 << 12;

#[derive(Debug)]
pub enum MdioError {
    Timeout,
    InvalidPhyAddress,
    InvalidRegister,
    BusError,
    DeviceNotFound,
}

pub type Result<T> = core::result::Result<T, MdioError>;
use core::ptr::{read_volatile, write_volatile};
use core::time::Duration;

pub struct Jh7110Mdio<H: super::DwmacHal> {
    base_addr: usize,
    _phantom: core::marker::PhantomData<H>,
}

impl<H: super::DwmacHal> Jh7110Mdio<H> {
    /// 创建新的MDIO控制器实例
    pub fn new(gmac_base: usize) -> Self {
        Self {
            base_addr: gmac_base,
            _phantom: core::marker::PhantomData,
        }
    }

    /// 等待MDIO操作完成
    fn wait_mdio_idle(&self) -> Result<()> {
        for _ in 0..100 {
            let addr_reg =
                unsafe { read_volatile((self.base_addr + MAC_MDIO_ADDRESS_OFFSET) as *const u32) };

            if (addr_reg & MAC_MDIO_ADDRESS_GB) == 0 {
                return Ok(());
            }
            H::wait_until(Duration::from_millis(10));
        }

        Err(MdioError::Timeout)
    }
}

impl<H: super::DwmacHal> Jh7110Mdio<H> {
    /// 执行Clause 22 PHY寄存器读操作
    pub fn read_c22(&self, phy_addr: u8, reg_addr: u16) -> Result<u16> {
        if phy_addr > 31 {
            return Err(MdioError::InvalidPhyAddress);
        }

        // 等待MDIO空闲
        self.wait_mdio_idle()?;

        // 构建地址寄存器值
        let addr_value = ((phy_addr as u32) << MAC_MDIO_ADDRESS_PA_SHIFT)
            | ((reg_addr as u32) << MAC_MDIO_ADDRESS_RDA_SHIFT)
            | (MAC_MDIO_ADDRESS_CR << MAC_MDIO_ADDRESS_CR_SHIFT)
            | (MAC_MDIO_ADDRESS_GOC_READ << MAC_MDIO_ADDRESS_GOC_SHIFT)
            | MAC_MDIO_ADDRESS_GB;

        // 写入地址寄存器触发读操作
        unsafe {
            write_volatile(
                (self.base_addr + MAC_MDIO_ADDRESS_OFFSET) as *mut u32,
                addr_value,
            );
        }

        // 等待操作完成
        self.wait_mdio_idle()?;

        // 读取数据
        let data = unsafe { read_volatile((self.base_addr + MAC_MDIO_DATA_OFFSET) as *const u32) };

        Ok(data as u16)
    }

    /// 执行Clause 22 PHY寄存器写操作
    pub fn write_c22(&self, phy_addr: u8, reg_addr: u16, data: u16) -> Result<()> {
        if phy_addr > 31 {
            return Err(MdioError::InvalidPhyAddress);
        }

        // 等待MDIO空闲
        self.wait_mdio_idle()?;

        // 先写入数据
        unsafe {
            write_volatile(
                (self.base_addr + MAC_MDIO_DATA_OFFSET) as *mut u32,
                data as u32,
            );
        }

        // 构建地址寄存器值
        let addr_value = ((phy_addr as u32) << MAC_MDIO_ADDRESS_PA_SHIFT)
            | ((reg_addr as u32) << MAC_MDIO_ADDRESS_RDA_SHIFT)
            | (MAC_MDIO_ADDRESS_CR << MAC_MDIO_ADDRESS_CR_SHIFT)
            | (MAC_MDIO_ADDRESS_GOC_WRITE << MAC_MDIO_ADDRESS_GOC_SHIFT)
            | MAC_MDIO_ADDRESS_GB;

        // 写入地址寄存器触发写操作
        unsafe {
            write_volatile(
                (self.base_addr + MAC_MDIO_ADDRESS_OFFSET) as *mut u32,
                addr_value,
            );
        }

        // 等待操作完成
        self.wait_mdio_idle()
    }
}

pub struct Yt8531cPhy<H: super::DwmacHal> {
    mdio: Jh7110Mdio<H>,
    phy_addr: u8,
}

impl<H: super::DwmacHal> Yt8531cPhy<H> {
    /// 创建YT8531C PHY实例
    pub fn new(gmac_base: usize, phy_addr: u8) -> Self {
        Self {
            mdio: Jh7110Mdio::new(gmac_base),
            phy_addr,
        }
    }

    /// 读取YT8531C扩展寄存器
    pub fn read_ext_reg(&self, ext_reg: u16) -> Result<u16> {
        // 第一步：写入扩展寄存器地址到0x1e
        self.mdio
            .write_c22(self.phy_addr, REG_DEBUG_ADDR_OFFSET, ext_reg)?;

        // 第二步：从0x1f读取数据
        self.mdio.read_c22(self.phy_addr, REG_DEBUG_DATA)
    }

    /// 写入YT8531C扩展寄存器
    pub fn write_ext_reg(&self, ext_reg: u16, data: u16) -> Result<()> {
        // 第一步：写入扩展寄存器地址到0x1e
        self.mdio
            .write_c22(self.phy_addr, REG_DEBUG_ADDR_OFFSET, ext_reg)?;

        // 第二步：写入数据到0x1f
        self.mdio.write_c22(self.phy_addr, REG_DEBUG_DATA, data)
    }

    /// 读取标准寄存器
    pub fn read_reg(&self, reg: u16) -> Result<u16> {
        self.mdio.read_c22(self.phy_addr, reg)
    }

    /// 写入标准寄存器
    pub fn write_reg(&self, reg: u16, data: u16) -> Result<()> {
        self.mdio.write_c22(self.phy_addr, reg, data)
    }

    /// 获取PHY ID
    pub fn get_phy_id(&self) -> Result<u32> {
        let id1 = self.read_reg(YT8531C_PHYID1)?;
        let id2 = self.read_reg(YT8531C_PHYID2)?;
        Ok(((id1 as u32) << 16) | (id2 as u32))
    }

    /// 软复位PHY
    pub fn soft_reset(&self) -> Result<()> {
        let mut bmcr = self.read_reg(YT8531C_BMCR)?;
        bmcr |= 0x8000; // 设置复位位
        self.write_reg(YT8531C_BMCR, bmcr)?;

        // 等待复位完成
        for _ in 0..100 {
            let bmcr = self.read_reg(YT8531C_BMCR)?;
            if (bmcr & 0x8000) == 0 {
                return Ok(());
            }
            H::wait_until(Duration::from_millis(10));
        }

        Err(MdioError::Timeout)
    }

    pub fn set_phy_linus(&self) -> Result<()> {
        log::info!("🔧 Setting PHY to Linux configuration...");
        self.soft_reset()?;
        self.write_ext_reg(0xa001, 0x7960)?;
        self.write_ext_reg(0xa003, 0xe91b)?;
        self.write_ext_reg(0xa005, 0xc5e1)?;
        self.write_ext_reg(0xa007, 0x2801)?;
        self.write_ext_reg(0xa009, 0x0300)?;
        self.write_ext_reg(0xa00b, 0x0000)?;
        self.write_ext_reg(0xa00d, 0x4007)?;
        self.write_ext_reg(0xa00f, 0x2000)?;

        Ok(())
    }

    pub fn configure_rgmii_id(&mut self) -> Result<()> {
        log::info!("🔧 Configuring YT8531 RGMII-ID mode...");

        // 1. 配置RGMII延迟参数
        self.configure_rgmii_delays()?;

        // 2. 配置Motorcomm特定功能
        self.configure_motorcomm_features()?;

        // 3. 启用RX时钟门控
        self.enable_rx_clock_gating()?;

        log::info!("✅ RGMII-ID configuration completed");
        Ok(())
    }

    fn configure_rgmii_delays(&self) -> Result<()> {
        // VisionFive2 v1.3b推荐延迟值
        let rx_delay_ps = 1900u32; // RX延迟1900ps
        let tx_delay_ps = 1350u32; // TX延迟1350ps

        // 查找延迟值对应的寄存器值
        let rx_reg_val = 0xd; // 默认1900ps对应0xD

        let tx_reg_val = 0x9; // 默认1350ps对应0x9

        // 读取当前配置
        let mut config_reg = self.read_ext_reg(YT8531C_EXT_CHIP_CONFIG)?;
        let mut rgmii_reg = self.read_ext_reg(YT8531C_EXT_RGMII_CONFIG1)?;

        // 配置RX延迟
        config_reg |= YT8531_CCR_RXC_DLY_EN; // 启用RX延迟
        rgmii_reg &= !YT8531_RC1R_RX_DELAY_MASK;
        rgmii_reg |= (rx_reg_val << 12) & YT8531_RC1R_RX_DELAY_MASK;

        // 配置TX延迟
        rgmii_reg &= !YT8531_RC1R_GE_TX_DELAY_MASK;
        rgmii_reg |= (tx_reg_val << 8) & YT8531_RC1R_GE_TX_DELAY_MASK;

        // 写回寄存器
        self.write_ext_reg(YT8531C_EXT_CHIP_CONFIG, config_reg)?;
        self.write_ext_reg(YT8531C_EXT_RGMII_CONFIG1, rgmii_reg)?;

        log::info!(
            "   📊 RX delay: {}ps (reg: 0x{:X})",
            rx_delay_ps,
            rx_reg_val
        );
        log::info!(
            "   📊 TX delay: {}ps (reg: 0x{:X})",
            tx_delay_ps,
            tx_reg_val
        );

        Ok(())
    }

    fn configure_motorcomm_features(&self) -> Result<()> {
        // 配置Motorcomm YT8531特定功能[2]
        // 这些配置对应Linux设备树中的motorcomm属性

        // 启用TX时钟调整
        // 对应 motorcomm,tx-clk-adj-enabled
        let mut ext_reg = self.read_ext_reg(YT8531C_EXT_SYNCE_CFG)?;
        ext_reg |= 1 << 8; // 启用TX时钟调整
        self.write_ext_reg(YT8531C_EXT_SYNCE_CFG, ext_reg)?;

        // 配置100M TX时钟反转
        // 对应 motorcomm,tx-clk-100-inverted
        let mut clk_config = self.read_ext_reg(YT8531C_EXT_CLK_TX_INVERT)?;
        clk_config |= 1 << 4; // 100M TX时钟反转
        self.write_ext_reg(YT8531C_EXT_CLK_TX_INVERT, clk_config)?;

        // 配置1000M TX时钟反转
        // 对应 motorcomm,tx-clk-1000-inverted
        clk_config |= 1 << 5; // 1000M TX时钟反转
        self.write_ext_reg(YT8531C_EXT_CLK_TX_INVERT, clk_config)?;

        log::info!("   🔧 Motorcomm specific features configured");
        Ok(())
    }

    fn enable_rx_clock_gating(&mut self) -> Result<()> {
        // 启用RX时钟门控以节省功耗[10]
        let mut clock_gating = self.read_ext_reg(YT8531C_EXT_CLK_OUTPUT)?;
        clock_gating |= YT8531_CGR_RX_CLK_EN;
        self.write_ext_reg(YT8531C_CLOCK_GATING_REG, clock_gating)?;

        log::info!("   ⚡ RX clock gating enabled");
        Ok(())
    }
}

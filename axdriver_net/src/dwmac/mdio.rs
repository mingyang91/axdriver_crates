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
pub const REG_DEBUG_ADDR_OFFSET: u16 = 0x1e; // 扩展寄存器地址寄存器
pub const REG_DEBUG_DATA: u16 = 0x1f; // 扩展寄存器数据寄存器

// YT8531C扩展寄存器地址
pub const YT8531C_EXT_SLEEP_CONTROL1: u16 = 0x0027;
pub const YT8531C_EXT_CLK_OUTPUT: u16 = 0xa001;
pub const YT8531C_EXT_CHIP_CONFIG: u16 = 0xa001;
pub const YT8531C_EXT_SYNCE_CFG: u16 = 0xa012;

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
}

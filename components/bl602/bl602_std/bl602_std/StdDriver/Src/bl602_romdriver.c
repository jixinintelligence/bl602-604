#include "bl602_romdriver.h"
#include <string.h>

/** @addtogroup  BL602_Periph_Driver
 *  @{
 */

/** @defgroup ROMDRIVER
 *  @brief ROMDRIVER common functions
 *  @{
 */

/** @defgroup ROMDRIVER_Private_Type
 *  @{
 */
uint32_t const romDriverTable[]={
    0x06020001,
    0x00000000,
    0x00000000,
    0x00000000,
    
    [ROM_API_INDEX_AON_Power_On_MBG]     = (uint32_t)AON_Power_On_MBG,     /* checked (0x4000F108), */
    [ROM_API_INDEX_AON_Power_Off_MBG]    = (uint32_t)AON_Power_Off_MBG,    /* checked */
    [ROM_API_INDEX_AON_Power_On_XTAL]    = (uint32_t)AON_Power_On_XTAL,    /* checked (0x4000F108), */
    [ROM_API_INDEX_AON_Set_Xtal_CapCode] = (uint32_t)AON_Set_Xtal_CapCode, /* checked (0x4000F108), */
    [ROM_API_INDEX_AON_Power_Off_XTAL]   = (uint32_t)AON_Power_Off_XTAL,   /* checked */

    [ROM_API_INDEX_ASM_Delay_Us]                        = (uint32_t)ASM_Delay_Us,   /* checked */
    [ROM_API_INDEX_BL602_Delay_US]                      = (uint32_t)BL602_Delay_US, /* checked (0x4000F108), */
    [ROM_API_INDEX_BL602_Delay_MS]                      = (uint32_t)BL602_Delay_MS, /* checked (0x4000F108), verified */
    [ROM_API_INDEX_BL602_MemCpy]                        = (uint32_t)BL602_MemCpy,      /* checked */
    [ROM_API_INDEX_BL602_MemCpy4]                       = (uint32_t)BL602_MemCpy4,     /* checked */
    [ROM_API_INDEX_BL602_MemCpy_Fast]                   = (uint32_t)BL602_MemCpy_Fast, /* checked */
    [ROM_API_INDEX_BL602_MemSet]                        = (uint32_t)BL602_MemSet,  /* checked, checked */
    [ROM_API_INDEX_BL602_MemSet4]                       = (uint32_t)BL602_MemSet4, /* checked */
    [ROM_API_INDEX_BL602_MemCmp]                        = (uint32_t)BL602_MemCmp,  /* checked, checked */
    [ROM_API_INDEX_BFLB_Soft_CRC32]                     = (uint32_t)BFLB_Soft_CRC32, /* checked */
    [ROM_API_INDEX_GLB_Get_Root_CLK_Sel]                = (uint32_t)GLB_Get_Root_CLK_Sel,   /* checked */
    [ROM_API_INDEX_GLB_Set_System_CLK_Div]              = (uint32_t)GLB_Set_System_CLK_Div, /* checked (0x4000F108), */
    [ROM_API_INDEX_Update_SystemCoreClockWith_XTAL]     = (uint32_t)Update_SystemCoreClockWith_XTAL, /* checked (0x4000F108), */
    [ROM_API_INDEX_GLB_Set_System_CLK]                  = (uint32_t)GLB_Set_System_CLK,    /* checked, verified */
    [ROM_API_INDEX_System_Core_Clock_Update_From_RC32M] = (uint32_t)System_Core_Clock_Update_From_RC32M, /* checked */
    [ROM_API_INDEX_GLB_Set_SF_CLK]                      = (uint32_t)GLB_Set_SF_CLK,        /* checked, checked */
    [ROM_API_INDEX_GLB_SW_System_Reset]                 = (uint32_t)GLB_SW_System_Reset,   /* checked */
    [ROM_API_INDEX_GLB_SW_CPU_Reset]                    = (uint32_t)GLB_SW_CPU_Reset,      /* checked */
    [ROM_API_INDEX_GLB_SW_POR_Reset]                    = (uint32_t)GLB_SW_POR_Reset,      /* checked */
    [ROM_API_INDEX_GLB_Select_Internal_Flash]           = (uint32_t)GLB_Select_Internal_Flash, /* checked */
    [ROM_API_INDEX_GLB_Swap_Flash_Pin]                  = (uint32_t)GLB_Swap_Flash_Pin,    /* checked */
    [ROM_API_INDEX_GLB_GPIO_Init]                       = (uint32_t)GLB_GPIO_Init,         /* checked */
    [ROM_API_INDEX_GLB_Deswap_Flash_Pin]                = (uint32_t)GLB_Deswap_Flash_Pin,   /* checked */
    [ROM_API_INDEX_GLB_Select_External_Flash]           = (uint32_t)GLB_Select_External_Flash, /* checked */
    [ROM_API_INDEX_GLB_GPIO_Get_Fun]                    = (uint32_t)GLB_GPIO_Get_Fun,      /* checked */
    [ROM_API_INDEX_EF_Ctrl_Busy]                        = (uint32_t)EF_Ctrl_Busy,          /* checked */
    [ROM_API_INDEX_EF_Ctrl_Sw_AHB_Clk_0]                = (uint32_t)EF_Ctrl_Sw_AHB_Clk_0,  /* checked */
    [ROM_API_INDEX_EF_Ctrl_Load_Efuse_R0]               = (uint32_t)EF_Ctrl_Load_Efuse_R0, /* checked */
    [ROM_API_INDEX_EF_Ctrl_Get_Trim_Parity]             = (uint32_t)EF_Ctrl_Get_Trim_Parity, /* checked */
    [ROM_API_INDEX_EF_Ctrl_Read_RC32K_Trim]             = (uint32_t)EF_Ctrl_Read_RC32K_Trim, /* checked */
    [ROM_API_INDEX_EF_Ctrl_Read_RC32M_Trim]             = (uint32_t)EF_Ctrl_Read_RC32M_Trim, /* checked */
    [ROM_API_INDEX_PDS_Trim_RC32M]                      = (uint32_t)PDS_Trim_RC32M,        /* checked */
    [ROM_API_INDEX_PDS_Select_RC32M_As_PLL_Ref]         = (uint32_t)PDS_Select_RC32M_As_PLL_Ref, /* checked */
    [ROM_API_INDEX_PDS_Select_XTAL_As_PLL_Ref]          = (uint32_t)PDS_Select_XTAL_As_PLL_Ref,  /* checked */
    [ROM_API_INDEX_PDS_Power_On_PLL]                    = (uint32_t)PDS_Power_On_PLL,            /* checked */
    [ROM_API_INDEX_PDS_Enable_PLL_All_Clks]             = (uint32_t)PDS_Enable_PLL_All_Clks,     /* checked */
    [ROM_API_INDEX_PDS_Disable_PLL_All_Clks]            = (uint32_t)PDS_Disable_PLL_All_Clks,    /* checked */
    [ROM_API_INDEX_PDS_Enable_PLL_Clk]                  = (uint32_t)PDS_Enable_PLL_Clk,          /* checked */
    [ROM_API_INDEX_PDS_Disable_PLL_Clk]                 = (uint32_t)PDS_Disable_PLL_Clk,         /* checked */
    [ROM_API_INDEX_PDS_Power_Off_PLL]                   = (uint32_t)PDS_Power_Off_PLL,           /* checked */
    [ROM_API_INDEX_HBN_Enable]                          = (uint32_t)HBN_Enable,                  /* checked */
    [ROM_API_INDEX_HBN_Reset]                           = (uint32_t)HBN_Reset,                   /* checked */
    [ROM_API_INDEX_HBN_GPIO7_Dbg_Pull_Cfg]              = (uint32_t)HBN_GPIO7_Dbg_Pull_Cfg,      /* checked */
    [ROM_API_INDEX_HBN_Trim_RC32K]                      = (uint32_t)HBN_Trim_RC32K,              /* checked */
    [ROM_API_INDEX_HBN_Set_ROOT_CLK_Sel]                = (uint32_t)HBN_Set_ROOT_CLK_Sel,        /* checked */
    
    [ROM_API_INDEX_XIP_SFlash_State_Save]               = (uint32_t)XIP_SFlash_State_Save,              /* checked */
    [ROM_API_INDEX_XIP_SFlash_State_Restore]            = (uint32_t)XIP_SFlash_State_Restore,           /* checked */
    [ROM_API_INDEX_XIP_SFlash_Erase_Need_Lock]          = (uint32_t)XIP_SFlash_Erase_Need_Lock,         /* checked */
    [ROM_API_INDEX_XIP_SFlash_Write_Need_Lock]          = (uint32_t)XIP_SFlash_Write_Need_Lock,         /* checked */
    [ROM_API_INDEX_XIP_SFlash_Read_Need_Lock]           = (uint32_t)XIP_SFlash_Read_Need_Lock,          /* checked */
    [ROM_API_INDEX_XIP_SFlash_GetJedecId_Need_Lock]     = (uint32_t)XIP_SFlash_GetJedecId_Need_Lock,    /* checked */
    [ROM_API_INDEX_XIP_SFlash_GetDeviceId_Need_Lock]    = (uint32_t)XIP_SFlash_GetDeviceId_Need_Lock,   /* checked */
    [ROM_API_INDEX_XIP_SFlash_GetUniqueId_Need_Lock]    = (uint32_t)XIP_SFlash_GetUniqueId_Need_Lock,   /* checked */
    [ROM_API_INDEX_XIP_SFlash_Read_Via_Cache_Need_Lock] = (uint32_t)XIP_SFlash_Read_Via_Cache_Need_Lock,/* checked */
    [ROM_API_INDEX_XIP_SFlash_Read_With_Lock]           = (uint32_t)XIP_SFlash_Read_With_Lock,          /* checked */
    [ROM_API_INDEX_XIP_SFlash_Write_With_Lock]          = (uint32_t)XIP_SFlash_Write_With_Lock,         /* checked */
    [ROM_API_INDEX_XIP_SFlash_Erase_With_Lock]          = (uint32_t)XIP_SFlash_Erase_With_Lock,         /* checked */

    [ROM_API_INDEX_SFlash_Init]                      = (uint32_t)SFlash_Init,       /* checked, verified */
    [ROM_API_INDEX_SFlash_SetSPIMode]                = (uint32_t)SFlash_SetSPIMode, /* checked */
    [ROM_API_INDEX_SFlash_Read_Reg]                  = (uint32_t)SFlash_Read_Reg,   /* checked (0x4000F108), */
    [ROM_API_INDEX_SFlash_Write_Reg]                 = (uint32_t)SFlash_Write_Reg,  /* checked (0x4000F108), */
    [ROM_API_INDEX_SFlash_Busy]                      = (uint32_t)SFlash_Busy,       /* checked (0x4000F108), */
    [ROM_API_INDEX_SFlash_Write_Enable]              = (uint32_t)SFlash_Write_Enable,  /* checked (0x4000F108), */
    [ROM_API_INDEX_SFlash_Qspi_Enable]               = (uint32_t)SFlash_Qspi_Enable,   /* checked (0x4000F108), verified */
    [ROM_API_INDEX_SFlash_Volatile_Reg_Write_Enable] = (uint32_t)SFlash_Volatile_Reg_Write_Enable, /* checked */
    [ROM_API_INDEX_SFlash_Chip_Erase]                = (uint32_t)SFlash_Chip_Erase,            /* checked, verified */
    [ROM_API_INDEX_SFlash_Sector_Erase]              = (uint32_t)SFlash_Sector_Erase,          /* checked */
    [ROM_API_INDEX_SFlash_Blk32_Erase]               = (uint32_t)SFlash_Blk32_Erase,           /* checked */
    [ROM_API_INDEX_SFlash_Blk64_Erase]               = (uint32_t)SFlash_Blk64_Erase,           /* checked */
    [ROM_API_INDEX_SFlash_Erase]                     = (uint32_t)SFlash_Erase,                 /* checked, verified */
    [ROM_API_INDEX_SFlash_Program]                   = (uint32_t)SFlash_Program,               /* checked, verified */
    [ROM_API_INDEX_SFlash_GetUniqueId]               = (uint32_t)SFlash_GetUniqueId,           /* checked */
    [ROM_API_INDEX_SFlash_GetJedecId]                = (uint32_t)SFlash_GetJedecId,            /* checked, verified */
    [ROM_API_INDEX_SFlash_GetDeviceId]               = (uint32_t)SFlash_GetDeviceId,           /* checked */
    [ROM_API_INDEX_SFlash_Powerdown]                 = (uint32_t)SFlash_Powerdown,             /* checked */
    [ROM_API_INDEX_SFlash_Releae_Powerdown]          = (uint32_t)SFlash_Releae_Powerdown,      /* checked */
    [ROM_API_INDEX_SFlash_SetBurstWrap]              = (uint32_t)SFlash_SetBurstWrap,          /* checked, verified */
    [ROM_API_INDEX_SFlash_DisableBurstWrap]          = (uint32_t)SFlash_DisableBurstWrap,      /* checked */
    [ROM_API_INDEX_SFlash_Software_Reset]            = (uint32_t)SFlash_Software_Reset,        /* checked */
    [ROM_API_INDEX_SFlash_Reset_Continue_Read]       = (uint32_t)SFlash_Reset_Continue_Read,   /* checked */
    [ROM_API_INDEX_SFlash_Set_IDbus_Cfg]             = (uint32_t)SFlash_Set_IDbus_Cfg,         /* checked */
    [ROM_API_INDEX_SFlash_IDbus_Read_Enable]         = (uint32_t)SFlash_IDbus_Read_Enable,     /* checked */
    [ROM_API_INDEX_SFlash_Cache_Enable_Set]          = (uint32_t)SFlash_Cache_Enable_Set,      /* checked (0x4000F108), verified */
    [ROM_API_INDEX_SFlash_Cache_Flush]               = (uint32_t)SFlash_Cache_Flush,           /* checked */
    [ROM_API_INDEX_SFlash_Cache_Read_Enable]         = (uint32_t)SFlash_Cache_Read_Enable,     /* checked */
    [ROM_API_INDEX_SFlash_Cache_Hit_Count_Get]       = (uint32_t)SFlash_Cache_Hit_Count_Get,   /* checked */
    [ROM_API_INDEX_SFlash_Cache_Miss_Count_Get]      = (uint32_t)SFlash_Cache_Miss_Count_Get,  /* checked */
    [ROM_API_INDEX_SFlash_Cache_Read_Disable]        = (uint32_t)SFlash_Cache_Read_Disable,    /* checked */
    [ROM_API_INDEX_SFlash_Read]                      = (uint32_t)SFlash_Read, /* checked, verified */

    [ROM_API_INDEX_SF_Ctrl_Enable]                 = (uint32_t)SF_Ctrl_Enable,         /* checked */
    [ROM_API_INDEX_SF_Ctrl_Set_Owner]              = (uint32_t)SF_Ctrl_Set_Owner,      /* checked */
    [ROM_API_INDEX_SF_Ctrl_Disable]                = (uint32_t)SF_Ctrl_Disable,        /* checked */
    [ROM_API_INDEX_SF_Ctrl_Select_Pad]             = (uint32_t)SF_Ctrl_Select_Pad,     /* checked */
    [ROM_API_INDEX_SF_Ctrl_AES_Enable_BE]          = (uint32_t)SF_Ctrl_AES_Enable_BE,  /* checked */
    [ROM_API_INDEX_SF_Ctrl_AES_Enable_LE]          = (uint32_t)SF_Ctrl_AES_Enable_LE,  /* checked */
    [ROM_API_INDEX_SF_Ctrl_AES_Set_Region]         = (uint32_t)SF_Ctrl_AES_Set_Region, /* checked */
    [ROM_API_INDEX_SF_Ctrl_AES_Set_Key]            = (uint32_t)SF_Ctrl_AES_Set_Key,    /* __REV */
    [ROM_API_INDEX_SF_Ctrl_AES_Set_Key_BE]         = (uint32_t)SF_Ctrl_AES_Set_Key_BE, /* checked */
    [ROM_API_INDEX_SF_Ctrl_AES_Set_IV]             = (uint32_t)SF_Ctrl_AES_Set_IV,     /* __REV */
    [ROM_API_INDEX_SF_Ctrl_AES_Set_IV_BE]          = (uint32_t)SF_Ctrl_AES_Set_IV_BE,  /* __REV */
    [ROM_API_INDEX_SF_Ctrl_AES_Enable]             = (uint32_t)SF_Ctrl_AES_Enable,     /* checked */
    [ROM_API_INDEX_SF_Ctrl_AES_Disable]            = (uint32_t)SF_Ctrl_AES_Disable,    /* checked */
    [ROM_API_INDEX_SF_Ctrl_Set_Flash_Image_Offset] = (uint32_t)SF_Ctrl_Set_Flash_Image_Offset,  /* checked */
    [ROM_API_INDEX_SF_Ctrl_Get_Flash_Image_Offset] = (uint32_t)SF_Ctrl_Get_Flash_Image_Offset,  /* checked */
    [ROM_API_INDEX_SF_Ctrl_Select_Clock]           = (uint32_t)SF_Ctrl_Select_Clock, /* checked */
    [ROM_API_INDEX_SF_Ctrl_SendCmd]                = (uint32_t)SF_Ctrl_SendCmd,      /* checked */
    [ROM_API_INDEX_SF_Ctrl_Icache_Set]             = (uint32_t)SF_Ctrl_Icache_Set,   /* checked */
    [ROM_API_INDEX_SF_Ctrl_Icache2_Set]            = (uint32_t)SF_Ctrl_Icache2_Set,  /* checked */
    [ROM_API_INDEX_SF_Ctrl_GetBusyState]           = (uint32_t)SF_Ctrl_GetBusyState, /* checked */
    [ROM_API_INDEX_SF_Cfg_Deinit_Ext_Flash_Gpio]   = (uint32_t)SF_Cfg_Deinit_Ext_Flash_Gpio, /* checked */
    [ROM_API_INDEX_SF_Cfg_Init_Ext_Flash_Gpio]     = (uint32_t)SF_Cfg_Init_Ext_Flash_Gpio,   /* checked */

    [ROM_API_INDEX_SF_Cfg_Restore_GPIO17_Fun]      = (uint32_t)SF_Cfg_Restore_GPIO17_Fun,      /* checked */
    [ROM_API_INDEX_SF_Cfg_Get_Flash_Cfg_Need_Lock] = (uint32_t)SF_Cfg_Get_Flash_Cfg_Need_Lock, /* checked */
    [ROM_API_INDEX_SF_Cfg_Init_Flash_Gpio]         = (uint32_t)SF_Cfg_Init_Flash_Gpio, /* checked */
    [ROM_API_INDEX_SF_Cfg_Flash_Identify]          = (uint32_t)SF_Cfg_Flash_Identify,  /* checked, verified */

    [ROM_API_INDEX_FUNC_INVALID_START ... ROM_API_INDEX_FUNC_LAST_ENTRY] = 0xdeedbeef,
};

/*@} end of group ROMDRIVER_Private_Type*/

/** @defgroup ROMDRIVER_Private_Defines
 *  @{
 */

/*@} end of group ROMDRIVER_Private_Defines */

/** @defgroup ROMDRIVER_Private_Variables
 *  @{
 */                               

/*@} end of group ROMDRIVER_Private_Variables */

/** @defgroup ROMDRIVER_Global_Variables
 *  @{
 */

/*@} end of group ROMDRIVER_Global_Variables */

/** @defgroup ROMDRIVER_Private_FunctionDeclaration
 *  @{
 */

/*@} end of group ROMDRIVER_Private_FunctionDeclaration */

/** @defgroup ROMDRIVER_Private_Functions
 *  @{
 */

/*@} end of group ROMDRIVER_Private_Functions */

/** @defgroup ROMDRIVER_Public_Functions
 *  @{
 */

/*@} end of group ROMDRIVER_Public_Functions */

/*@} end of group ROMDRIVER_COMMON */

/*@} end of group BL602_Periph_Driver */



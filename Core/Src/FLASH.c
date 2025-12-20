#include "FLASH.h"
void FLASH_Lock()
{
    HAL_FLASH_Lock();   
}
void FLASH_UnLock()
{
    HAL_FLASH_Unlock();

}

void FLASHEx_Erase(uint32_t addr)
{
    FLASH_EraseInitTypeDef pErase;
    pErase.NbPages = 1;
    pErase.PageAddress = addr;
    pErase.TypeErase = FLASH_TYPEERASE_PAGES; // xoa theo page
    uint32_t PError;
    HAL_FLASHEx_Erase( &pErase, &PError);
}

void FLash_write(uint32_t addr, uint8_t *mdata,uint16_t len)
{
    uint8_t status = 0x00;
    if(len%2 == 1)
    {
        status = 1;
        len --; 
    }
    for(uint32_t i =0 ;i<len ;i += 2)
    {
         HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, *(mdata+i)| (uint16_t)(*(mdata+i +1)<<8));
    }
    if(status)
    {
         HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + len, *(mdata+len)| ((uint16_t)(0xFF)<<8));
    }

}
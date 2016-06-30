/*
 * internal_flash_api.c
 *
 *  Created on: 11. Apr. 2016
 *      Author: martin.tomasini2
 */
#include "internal_flash_api.h"
#include "cmsis.h"


    #define NUMBER_OF_FLASH_SECTORS 8
    #define SECTOR_0_SIZE 16
    #define SECTOR_0_START_ADD 0x8000000
    #define SECTOR_0_END_ADD 0x8003FFFF

    #define SECTOR_1_SIZE 16
    #define SECTOR_1_START_ADD 0x8004000
    #define SECTOR_1_END_ADD 0x8007FFF

    #define SECTOR_2_SIZE 16
    #define SECTOR_2_START_ADD 0x8008000
    #define SECTOR_2_END_ADD 0x800BFFF

    #define SECTOR_3_SIZE 16
    #define SECTOR_3_START_ADD 0x800C000
    #define SECTOR_3_END_ADD 0x800FFFF

    #define SECTOR_4_SIZE 64
    #define SECTOR_4_START_ADD 0x8010000
    #define SECTOR_4_END_ADD 0x801FFFF

    #define SECTOR_5_SIZE 128
    #define SECTOR_5_START_ADD 0x8020000
    #define SECTOR_5_END_ADD 0x803FFFF

    #define SECTOR_6_SIZE 128
    #define SECTOR_6_START_ADD 0x8040000
    #define SECTOR_6_END_ADD 0x805FFFF

    #define SECTOR_7_SIZE 128
    #define SECTOR_7_START_ADD 0x8060000
    #define SECTOR_7_END_ADD 0x807FFFF

    #define FLASH_SECTOR_0_START_ADDRESS  0x8000000
    #define FLASH_LAST_SECTOR_END_ADDRESS 0x0807FFFF


#warning "Test internal flash driver for F401 devices!!!"

int WriteFlash(uint32_t ui32Address, const uint8_t* pData, uint32_t ui32DataSize)
{
    if((ui32Address > FLASH_LAST_SECTOR_END_ADDRESS) || (ui32Address < FLASH_SECTOR_0_START_ADDRESS))
    {
        return -2;
    }

    volatile uint16_t uintDataBuffer;

    // Check if it is 2 bytes aligned
    if(0 != (ui32Address & 1))
    {
        return -3;
    }

    HAL_FLASH_Unlock();

    // Write payload byte per byte
    for(uint32_t i = 0; i < ui32DataSize; i += 2, ui32Address += 2)
    {
        if(i < ui32DataSize - 1)
        {
            uintDataBuffer = pData[i];
            uintDataBuffer +=  (uint16_t)(pData[i + 1] << 8);
        }
        else
        {
            uintDataBuffer = pData[i];
            uintDataBuffer += 0xFF00;
        }
        if(HAL_OK !=  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ui32Address, (uint64_t)uintDataBuffer))
        {
            HAL_FLASH_Lock();

            return -1;
        }
    }

    HAL_FLASH_Lock();

    return 0;

}

int ReadFlash(uint32_t ui32Address, uint8_t* pData, uint32_t ui32DataSize)
{
    if((ui32Address > FLASH_LAST_SECTOR_END_ADDRESS) || (ui32Address < FLASH_SECTOR_0_START_ADDRESS))
    {
        return -2;
    }

    for(uint32_t i = 0; i < ui32DataSize; i++)
    {
        pData[i] = ((uint8_t*)ui32Address)[i];
    }

    return 0;
}

int EraseSectorByNumber(uint16_t ui16Sector)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t ui32PageError;

    if(ui16Sector < NUMBER_OF_FLASH_SECTORS)
    {
        HAL_FLASH_Unlock();

        EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector = ui16Sector;
        EraseInitStruct.NbSectors = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Todo: check if correct!!!!

        if (HAL_FLASHEx_Erase(&EraseInitStruct, &ui32PageError) != HAL_OK)
        {
            HAL_FLASH_Lock();

            return -1;
        }

        while(HAL_OK != FLASH_WaitForLastOperation((uint32_t)50000)){};

        HAL_FLASH_Lock();

        return 0;
    }

    // Sector does not exist
    return -2;
}

int EraseSectorByAddress(uint32_t ui32Address)
{
    if((ui32Address > FLASH_LAST_SECTOR_END_ADDRESS) || (ui32Address < FLASH_SECTOR_0_START_ADDRESS))
    {
        return -2;
    }

    if(SECTOR_1_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(0);
    }
    else if(SECTOR_2_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(1);
    }
    else if(SECTOR_3_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(2);
    }
    else if(SECTOR_4_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(3);
    }
    else if(SECTOR_5_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(4);
    }
    else if(SECTOR_6_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(5);
    }
    else if(SECTOR_7_START_ADD > ui32Address)
    {
        return EraseSectorByNumber(6);
    }
    else
    {
        return EraseSectorByNumber(7);
    }

}

int GetNumOfSectors(void)
{
    return NUMBER_OF_FLASH_SECTORS;
}

void GetSectorStartAdd(uint32_t ui32Sector, uint32_t* pui32Address)
{
    if(ui32Sector > NUMBER_OF_FLASH_SECTORS - 1)
    {
        return;// - 1;
    }
    switch(ui32Sector)
    {
        case 0:
            *pui32Address = SECTOR_0_START_ADD;
            break;
        case 1:
            *pui32Address = SECTOR_1_START_ADD;
            break;
        case 2:
            *pui32Address = SECTOR_2_START_ADD;
            break;
        case 3:
            *pui32Address = SECTOR_3_START_ADD;
            break;
        case 4:
            *pui32Address = SECTOR_4_START_ADD;
            break;
        case 5:
            *pui32Address = SECTOR_5_START_ADD;
            break;
        case 6:
            *pui32Address = SECTOR_6_START_ADD;
            break;
        case 7:
            *pui32Address = SECTOR_7_START_ADD;
            break;
    }
}

void GetSectorEndAdd(uint32_t ui32Sector, uint32_t* pui32Address)
{
    if(ui32Sector > NUMBER_OF_FLASH_SECTORS - 1)
    {
        return;// - 1;
    }
    switch(ui32Sector)
    {
        case 0:
            *pui32Address = SECTOR_0_END_ADD;
            break;
        case 1:
            *pui32Address = SECTOR_1_END_ADD;
            break;
        case 2:
            *pui32Address = SECTOR_2_END_ADD;
            break;
        case 3:
            *pui32Address = SECTOR_3_END_ADD;
            break;
        case 4:
            *pui32Address = SECTOR_4_END_ADD;
            break;
        case 5:
            *pui32Address = SECTOR_5_END_ADD;
            break;
        case 6:
            *pui32Address = SECTOR_6_END_ADD;
            break;
        case 7:
            *pui32Address = SECTOR_7_END_ADD;
            break;
    }
}



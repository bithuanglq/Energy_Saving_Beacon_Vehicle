################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.c 

OBJS += \
./Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.o 

COMPILED_SRCS += \
./Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.src 

C_DEPS += \
./Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/IfxGtm_Tim_In.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/%.src: ../Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc21x -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\CODE" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gpt12" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gpt12\IncrEnc" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gpt12\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\doc" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Configurations" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\_Build" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\_Impl" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\_Lib" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\_Lib\DataHandling" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\_Lib\InternalMux" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\_PinMap" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Asclin" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Asclin\Asc" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Asclin\Lin" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Asclin\Spi" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Asclin\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\Icu" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\PwmBc" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\PwmHl" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\Timer" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\TimerWithTrigger" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Ccu6\TPwm" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Cpu" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Cpu\CStart" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Cpu\Irq" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Cpu\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Cpu\Trap" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Dma" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Dma\Dma" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Dma\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Dts" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Dts\Dts" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Dts\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Flash" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Flash\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Tim" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Tim\In" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Tom" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Tom\Pwm" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Tom\PwmHl" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Tom\Timer" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Gtm\Trig" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Iom" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Iom\Driver" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Iom\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Mtu" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Mtu\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Multican" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Multican\Can" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Multican\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Port" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Port\Io" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Port\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Qspi" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Qspi\SpiMaster" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Qspi\SpiSlave" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Qspi\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Scu" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Scu\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Sent" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Sent\Sent" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Sent\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Smu" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Smu\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Src" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Src\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Stm" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Stm\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Stm\Timer" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Vadc" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Vadc\Adc" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\iLLD\TC22A\Tricore\Vadc\Std" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra\Platform" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra\Platform\Tricore" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra\Platform\Tricore\Compilers" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra\Sfr" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra\Sfr\TC22A" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Infra\Sfr\TC22A\_Reg" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\_Utilities" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\If" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\If\Ccu6If" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\StdIf" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe\Bsp" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe\Comm" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe\General" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe\Math" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe\Time" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\seekfree_libraries" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\seekfree_libraries\common" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\Libraries\seekfree_peripheral" -I"D:\ADS_WorkSpace\ConstantPower\TC212_Library\Seekfree_TC212_Opensource_Library\USER" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc21x -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\Seekfree_TC212_Opensource_Library\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/%.o: ./Libraries/infineon_libraries/iLLD/TC22A/Tricore/Gtm/Tim/In/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '



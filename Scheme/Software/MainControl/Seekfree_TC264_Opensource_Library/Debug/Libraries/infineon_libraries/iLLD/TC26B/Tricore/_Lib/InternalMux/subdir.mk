################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/Ifx_InternalMux.c 

OBJS += \
./Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/Ifx_InternalMux.o 

COMPILED_SRCS += \
./Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/Ifx_InternalMux.src 

C_DEPS += \
./Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/Ifx_InternalMux.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/%.src: ../Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb -I"E:\Infineon\Seekfree_TC264_Opensource_Library\CODE" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\doc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Configurations" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Build" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Impl" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Lib" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Lib\DataHandling" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_Lib\InternalMux" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\_PinMap" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Asc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Lin" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Spi" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Asclin\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\Icu" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\PwmBc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\PwmHl" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\Timer" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\TimerWithTrigger" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Ccu6\TPwm" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cif" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cif\Cam" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cif\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\CStart" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\Irq" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Cpu\Trap" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dma" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dma\Dma" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dma\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc\Dsadc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc\Rdc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dsadc\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dts" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dts\Dts" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Dts\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Emem" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Emem\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eray" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eray\Eray" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eray\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eth" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eth\Phy_Pef7071" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Eth\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fce" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fce\Crc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fce\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fft" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fft\Fft" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Fft\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Flash" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Flash\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gpt12" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gpt12\IncrEnc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gpt12\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom\Pwm" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom\PwmHl" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Atom\Timer" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tim" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tim\In" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom\Pwm" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom\PwmHl" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Tom\Timer" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Gtm\Trig" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Hssl" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Hssl\Hssl" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Hssl\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\I2c" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\I2c\I2c" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\I2c\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Iom" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Iom\Driver" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Iom\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Msc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Msc\Msc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Msc\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Mtu" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Mtu\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Multican" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Multican\Can" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Multican\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Port" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Port\Io" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Port\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5\Psi5" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5s" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5s\Psi5s" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Psi5s\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi\SpiMaster" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi\SpiSlave" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Qspi\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Scu" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Scu\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Sent" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Sent\Sent" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Sent\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Smu" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Smu\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Src" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Src\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Stm" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Stm\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Stm\Timer" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Vadc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Vadc\Adc" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\iLLD\TC26B\Tricore\Vadc\Std" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra\Platform" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra\Platform\Tricore" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra\Platform\Tricore\Compilers" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra\Sfr" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra\Sfr\TC26B" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Infra\Sfr\TC26B\_Reg" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Service" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\_Utilities" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\If" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\StdIf" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\infineon_libraries\Service\CpuGeneric\SysSe" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\seekfree_libraries" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\seekfree_libraries\common" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\Libraries\seekfree_peripheral" -I"E:\Infineon\Seekfree_TC264_Opensource_Library\USER" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1; sed -i -e '/ctc\\include/d' -e '/Libraries\\iLLD/d' -e '/Libraries\\Infra/d' -e 's/\(.*\)".*\\Seekfree_TC264_Opensource_Library\(\\.*\)"/\1\.\.\2/g' -e 's/\\/\//g' $(@:.src=.d) && \
	echo $(@:.src=.d) generated
	@echo 'Finished building: $<'
	@echo ' '

Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/%.o: ./Libraries/infineon_libraries/iLLD/TC26B/Tricore/_Lib/InternalMux/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '



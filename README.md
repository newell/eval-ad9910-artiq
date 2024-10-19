# Control AD9910 Evaluation Board with ARTIQ

The goal of this project is to control the [EVAL-AD9910](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/eval-ad9910.html) using [ARTIQ](https://github.com/m-labs/artiq).

## Synopsis

The EVAL-AD9910 contains an Altera MAX II EPM240T100C4N CPLD that is used to help control the AD9910 DDS on the EVAL-AD9910.  The specific EVAL-AD9910 I have doesn't have U1 populated and therefore there are some pins on the U9 header that I can use for SPI with the EBAZ4205 (for a similar setup see [Controlling the AD9834 DDS with the Zynq-SoC EBAZ4205 using ARTIQ](https://newell.github.io/projects/ebaz4205/)).  I will use a hacked version of [urukul](https://github.com/quartiq/urukul) to program the EPM240T100C4N where I plan to strip out the extra logic for the other DDS channels (only one DDS chip on the EVAL-AD9910).  

Here are the main steps:

* Create hacked gateware of the CPLD to use only one DDS channel
* Program CPLD with quartus
* Use ARTIQ and Urukul core device driver to control the DDS

  
# License

The license follows that of the [urukul](https://github.com/quartiq/urukul) project, GPLv3+.

## Short description

> NOTE:
>
>The project's name suggests that it should work on all WS281x types LEDs. That was primary intention, but presented driver has been tested only with WS2812B LEDs.


This repository contains driver for WS2812B LED controller where SPI peripheral is using to handle transmission timing. It is written in C++11 and it is using [*bit banding*](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337h/Behcjiic.html) and DMA to achieve two things:

 - relative small memory usage for storing, manipulating and transfering colors for each LED in the chain
 - using hardware during transmission as often as possible (unfortunately not in 100%)

As a result, the current version to store RGB colors and to transmit them to LEDs requires memory of size:

 - 2 x LEDs count x 3 bytes
 Two buffers to store each LED color in the chain of size *LEDs Count*:
    - multiplying by 2 because during ongoing transmission the driver operates on the second buffer
    - multiplying by 3 to store LEDs colors
 - 2 x 12 bytes

 These buffers are being used during transmission and each contain prepared RGB color for one LED at the time. DMA copies these bytes to SPI and *end of transfer interrupt* is used to convert color for next LED in chain. So, this is not fully hardware transmission, but interrupt handling that prepares next LED data to transmit takes about 6µs while CPU is running with 48 MHz clock.
 - 2 x (12 x 2 x *size of pointer*) bytes

  These tables store *bit banding* regions that correspond to specific bits for each transmitting buffer's byte.


## Detailed description


### Driver implementation
* #### SPI
Using SPI peripheral to drive WS281x LEDs is a well known solution. The idea is simple, each transmitted byte through SPI interface contains two bits of color, so to transmit one RGB color for one LED, there is need to send 12 'encoded' bytes.
Because 4 bits during SPI transmission represents one bit in WS281x interface, care must be taken while configuring clock for SPI peripheral:
    - transmission frequency of one bit of WS281x interface must be as close as to 800 kHz - but taking into consideration tolerances, acceptable transmission frequencies are in the range from 646 kHz to 1.05 MHz.
    - for example, using 48 MHz to clock SPI peripheral and using prescaler of value 16, each bit will be transmitted with 3MHz frequency. Because 4 bits sent by SPI are just one bit of WS281x's color, so the average frequency for transmitting one bit of LED color is 750kHz.

* #### Bit-banding
[Bit banding is well explained here.](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337h/Behcjiic.html)

  Two buffers - each 12 bytes long - are being used to transmit colors to LEDs. In the table below I have marked positions of two bits of color inside one transmitting byte. Transmitting bits 7:4 represents one bit of transmitting color and transmitting bits 3:0 represents next bit of color. If bit marked as 'X' is set to '0' then WS281x controller reads bits 7:4 or 3:0 as color's bit '0', otherwise reads them as '1'.

  <table>
    <tr>
        <td>bit</td>
        <td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td>
    </tr>
    <tr>
    	<td>value</td>
        <td>1</td><td><b style="color:green;">X</b></td><td>0</td><td>0</td><td>1</td><td><b style="color:green;">X</b></td><td>0</td><td>0</td>
    </tr>
  </table>


  Presented implementation prepares the table with pointers to bit band regions for 2nd and 6th bit for each byte inside transmitting buffers. Thanks to that writing under appropriate pointer any value greater than 0 sets corresponding bit to '1', and writing zero sets corresponding bit to '0'. Because writing operation to bit banding region is just like STR instruction, so it is quite fast.



* #### DMA

  Current implementation is using two DMA channels:
    1. Prior to send any color, need to convert RGB data into transmitting buffers. Like mentioned at the beginning, one LED color (RGB) is converted and stored inside of 12 bytes inside the transmitting buffer. Just before transmission is being started, the application converts colors of two LEDs and places them into transmitting buffers. Then transmission is started and 12 bytes of first buffer are being sent to the SPI peripheral by the DMA. During *each end of transfer interrupt* the next transmission (from the second buffer) is being started only if _end of transmission flag_ is not set. Next, still inside interrupt routine, checks if there is any color to send - if is, then RGB value is converted and stored inside of just freed transmitting buffer.
    2. There are two buffers which store colors for LEDs. Driver always uses one of them at the time. When *send()* was called then the driver does:
        - buffer which was just modified will be used as colors source during transmission - it is just 'renamed' to *transmitting* buffer and the second buffer (which was in the background) will be 'renamed' to *active* buffer, and there changes made by called API methods will be applied since now.
        - synchronizing these two buffers to have the same content - this ensures that next call to API that modifies LEDs colors will operate on the same content as was before calling *send()* method. That operation is done by the second channel of the DMA. As the result, the application can modify LED's colors while transmission of the previous 'snapshot' is in progress.



### PC visualization application

When I've implemented driver to version when it worked quite stable with a few animations running on 300 LEDs strip, I have put hardware to work continuously as a decoration. I have had to figure out how to test new code with lack of fully working hardware. The idea was to separate driver layer from transmission layer, so the PC application has to just simulate transmission process and LEDs. PC visualization application has been made with Qt5 and C++14 as an QtCreator4 project.

When main thread (GUI) is being started it constructs QWidget with small squares which are being painted horizontally. Each square represents one LED. Next, the new thread is created which uses WS281x driver API to run animations (that thread is equivalent to microcontroller program). The PC's transmission layer is just a simple method that runs in the another thread. Its job is send colors bytes provided by the WS281x driver to the main (GUI) thread and simulates transmission time.

PC visualization application sources are stored in [pc\_visualization\_proj](./pc_visualization_proj) subdirectory.

### Voltage converter - simple KiCAD project.

To ensure proper data signal shape (conversion level 3.3V -> 5V) I've designed simple hardware using parts that I already had on the 'shelf'. It uses two CMOS NOT gates connected in series and powered from 5V. Input signal from MCU is connected to the first gate through capacitor placed in series. Project files are stored in [KiCAD\_ws281x\_level\_converter](./KiCAD_ws281x_level_converter) subdirectory.

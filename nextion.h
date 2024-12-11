/*

Created by Simon Vizner 2024

*/

// DONT CHANGE!!!! The maximum returning data size in bytes
#define NX_RETURN_BUFF_SIZE 20

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

enum NXReturn {
    // Return Codes dependent on bkcmd value being greater than 0

    NX_INVALID_INSTRUCTION = 0x00,
    NX_INSTRUCTION_SUCCESSFUL = 0x01,
    NX_INVALID_COMPONENT_ID = 0x02,
    NX_INVALID_PAGE_ID = 0x03,
    NX_INVALID_PICTURE_ID = 0x04,
    NX_INVALID_FONT_ID = 0x05,
    NX_INVALID_FILE_OPERATION = 0x06,
    NX_INVALID_CRC = 0x09,
    NX_INVALID_BAUD_RATE_SETTING = 0x11,
    NX_INVALID_WAVEFORM_ID_OR_CHANNEL = 0x12,
    NX_INVALID_VARIABLE_NAME_OR_ATTRIBUTE = 0x1A,
    NX_INVALID_VARIABLE_OPERATION = 0x1B,
    NX_ASSIGNMENT_FAILED = 0x1C,
    NX_EEPROM_OPERATION_FAILED = 0x1D,
    NX_INVALID_QUANTITY_OF_PARAMETERS = 0x1E,
    NX_IO_OPERATION_FAILED = 0x1F,
    NX_ESCAPE_CHARACTER_INVALID = 0x20,
    NX_VARIABLE_NAME_TOO_LONG = 0x23,

    // Return Codes not affected by bkcmd value, valid in all cases

    NX_NEXTION_STARTUP = 0x00,
    NX_SERIAL_BUFFER_OVERFLOW = 0x24,
    NX_TOUCH_EVENT = 0x65,
    NX_CURRENT_PAGE_NUMBER = 0x66,
    NX_TOUCH_COORDINATE_AWAKE = 0x67,
    NX_TOUCH_COORDINATE_SLEEP = 0x68,
    NX_STRING_DATA_ENCLOSED = 0x70,
    NX_NUMERIC_DATA_ENCLOSED = 0x71,
    NX_AUTO_ENTERED_SLEEP_MODE = 0x86,
    NX_AUTO_WAKE_FROM_SLEEP = 0x87,
    NX_NEXTION_READY = 0x88,
    NX_START_MICROSD_UPGRADE = 0x89,
    NX_TRANSPARENT_DATA_FINISHED = 0xFD,
    NX_TRANSPARENT_DATA_READY = 0xFE
};

/*
Read returning data from the Nextion display into buffer

- <buffer> - The buffer that the function will store data in
    - You can create and allocate this buffer using `nx_alloc_buff()`
- returns `size_t` size of read bytes

Usage:
```c
uint8_t *buff = nx_alloc_buff();
size_t buff_size = nx_read(buff);

// Do whatever with the data
// Don't forget to free the buffer at the end
free(buff);
```
*/
size_t nx_read(uint8_t *buffer);

/*
Allocate buffer with big enough size for the nextion return data

- returns `uint8_t *` array of bytes

Usage:
```c
uint8_t *buff = nx_alloc_buff();

// Use the buff for `nx_read()` or something other
// Don't forget to free the buffer at the end
free(buff);
```
*/
uint8_t* nx_alloc_buff(void);

/*
Used to clean up the buffer, setting everyting to 0x00

- <buff> - Buffer to clean
- <size> - Size of the buffer

Usage:
```c
uint8_t *buff = nx_alloc_buff();
size_t buff_size = nx_read(buff);

// Do something with the data
// Let's say you want to use the buffer again
// so you need to clean it
nx_clean_buff(buff, buff_size);

// Do whatever with the clean buffer
// Don't forget to free the buffer at the end
free(buff);
```
*/
void nx_clean_buff(uint8_t *buff, size_t size);

/*
Write buffer with size to nextion display

- <bytes> - Buffer to write
- <size> - Size of the buffer 

Usage:
```c
uint8_t buff[] = {0xFF, 0xFF, 0xFF}

// We write the buff of size 3 to the nextion
nx_write(buff, 3);
```
*/
void nx_write(uint8_t *bytes, size_t size);

/*
Checks if returned buffer from nextion has the `0xFF 0xFF 0xFF` termination

Used internally within the library
*/
bool nx_has_termination(uint8_t *bytes, size_t size);

/*
Send the `0xFF 0xFF 0xFF` termination to the nexion
*/
void nx_send_term();

/*
Send a command to display an error code on the nextion 
*/
void nx_send_err(uint8_t code);

/*
Send a command to nextion

- <str> - Command to be sent

Usage:
```c
// This will send the `cls WHITE` command that clears the screen 
nx_send("cls WHITE");

// You can send any command with this funciton
nx_send("...")
```
*/
void nx_send(const char* str, ...);

/*
Send a command and read the response

- <buff> - Buffer that the function will store the data in
- <str> - Command to be sent
- returns 'size_t' size of the bytes read

Usage:
```c
uint8_t *buff = nx_alloc_buff();

// Will send the `sendme` command that returns the current page no.
size_t buff_size = nx_send_read(buff, "sendme");

// `buff[0]` will be equal to `0x66` which is the return code for the current page
// `buff[1]` will be equal to the page no.
```
*/
size_t nx_send_read(uint8_t *buff, const char *str, ...);

struct {
    uint8_t id;
    uint8_t page_n;
    uint8_t on_release;
    void (*trigger)(uint8_t*, size_t);
} typedef OnTouchFunc;

/*
Register a func to be executed when button with id and on page_n is touched

- <id> - Id of the button
- <page_n> - Page no. where the button is
- <trigger> - Function to be executed

Usage:
```c
// Let's say we have a button with id 1 on page 0
// We want to execute the `do_something()` function when the button is pressed

// The `trigger` func must be of type `void` with no arguments
void do_something(void) {
    //... you do something here
}

// We 'register' the function like so
nx_on_touch(1, 0, do_something);

// However if we want this to work, we need to call `nx_check()` func in an infinite loop
while(1) {
    nx_check(NULL, 0);
}
```
*/
void nx_on_touch(uint8_t id, uint8_t page_n, void (*trigger)(uint8_t*, size_t));

/*
Register a func to be executed when button with id and on page_n is released

- <id> - Id of the button
- <page_n> - Page no. where the button is
- <trigger> - Function to be executed

Usage:
```c
// Let's say we have a button with id 1 on page 0
// We want to execute the `do_something()` function when the button is released

// The `trigger` func must be of type `void` with no arguments
void do_something(void) {
    //... you do something here
}

// We 'register' the function like so
nx_on_release(1, 0, do_something);

// However if we want this to work, we need to call `nx_check()` func in an infinite loop
while(1) {
    nx_check(NULL, 0);
}
```
*/
void nx_on_release(uint8_t id, uint8_t page_n, void (*trigger)(uint8_t*, size_t));

/*
This function is mandatory for the functionalilty of the whole thing
It checks if there is any response from nextion
If there is, it processed the data
The parameters are optional and serve other purposes, don't use them if you don't know what you are doing

Either way, just use it as
```c
while(1){
    // ... some your code
    nx_check(NULL, 0);
}
```
*/
int* nx_check(uint8_t *buff, size_t buff_size);

/*
Returns the current page of the nextion
*/
uint8_t nx_current_page(void);

/*
Initializes the functionality and checks if the nextion is ready
*/
void nx_init(void);

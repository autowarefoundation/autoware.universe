/////////////////////////////////////////////////////////////////////////////
//
//! @file    mip_sdk_user_functions.c
//! @author  Nathan Miller
//! @version 1.1
//
//! @description Target-Specific Functions Required by the MIP SDK
//
// External dependencies:
//
//  mip.h
//
//!@copyright 2014 Lord Microstrain Sensing Systems.
//
//!@section CHANGES
//!
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
////////////////////////////////////////////////////////////////////////////////

#include "mip_sdk_user_functions.h"
#include <windows.h>

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_open(void *port_handle, int port_num, int baudrate)
//
//! @section DESCRIPTION
//! Target-Specific communications port open.
//
//! @section DETAILS
//!
//! @param [out] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in] int port_num       - port number (as recognized by the operating system.)
//! @param [in] int baudrate       - baudrate of the com port.
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem opening the port.\n
//! @retval MIP_USER_FUNCTION_OK     The open was successful.\n
//
//! @section NOTES
//!
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_open(void** port_handle, int port_num, int baudrate) {
  DWORD thread_id;
  CHAR port_name[100] = {0};
  CHAR port_index[100] = {0};
  BOOL ready;
  DCB dcb;
  HANDLE handle;

  // Create the port name
  strcat(port_name, "\\\\.\\COM");
  _itoa(port_num, port_index, 10);
  strcat(port_name, port_index);

  // Connect to the provided com port
  handle = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
  *port_handle = handle;

  // Check for an invalid handle
  if (*port_handle == INVALID_HANDLE_VALUE || *port_handle == 0) {
    return MIP_USER_FUNCTION_ERROR;
  }

  // Setup the com port buffer sizes
  if (SetupComm(*port_handle, MIP_COM_PORT_BUFFER_SIZE, MIP_COM_PORT_BUFFER_SIZE) == 0) {
    return MIP_USER_FUNCTION_ERROR;
  }

  // Setup the com port parameters
  ready = GetCommState(*port_handle, &dcb);

  // Close the serial port, mutex, and exit
  if (!ready) {
    return MIP_USER_FUNCTION_ERROR;
  }

  dcb.BaudRate = baudrate;    // Baudrate is typically 115200
  dcb.ByteSize = 8;           // Charsize is 8,  default for MicroStrain
  dcb.Parity = NOPARITY;      // Parity is none, default for MicroStrain
  dcb.StopBits = ONESTOPBIT;  // Stopbits is 1,  default for MicroStrain
  dcb.fAbortOnError = TRUE;
  ready = SetCommState(*port_handle, &dcb);

  // Close the serial port and exit
  if (!ready) {
    return MIP_USER_FUNCTION_ERROR;
  }

  return MIP_USER_FUNCTION_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_close(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific port close function
//
//! @section DETAILS
//!
//! @param [in] void *port_handle - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem closing the port.\n
//! @retval MIP_USER_FUNCTION_OK     The close was successful.\n
//
//! @section NOTES
//!
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_close(void* port_handle) {
  HANDLE handle;

  if (port_handle == NULL) return MIP_USER_FUNCTION_ERROR;

  handle = (HANDLE)(port_handle);

  // Close the serial port
  CloseHandle(handle);

  return MIP_USER_FUNCTION_OK;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_write(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_written, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_written - the number of bytes actually written to the port
//! @param [in]  u32 timeout_ms     - the write timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The write was successful.\n
//
//! @section NOTES
//!
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_write(void* port_handle, u8* buffer, u32 num_bytes, u32* bytes_written, u32 timeout_ms) {
  HANDLE handle;
  DWORD local_bytes_written;

  *bytes_written = 0;

  // Check for a valid port handle
  if (port_handle == NULL) return MIP_USER_FUNCTION_ERROR;

  handle = (HANDLE)(port_handle);

  // Call the windows write function
  if (WriteFile(handle, buffer, num_bytes, &local_bytes_written, NULL)) {
    *bytes_written = local_bytes_written;
    if (*bytes_written == num_bytes)
      return MIP_USER_FUNCTION_OK;
    else
      return MIP_USER_FUNCTION_ERROR;
  }

  return MIP_USER_FUNCTION_ERROR;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u16 mip_sdk_port_read(void *port_handle, u8 *buffer, u32 num_bytes, u32 *bytes_read, u32 timeout_ms)
//
//! @section DESCRIPTION
//! Target-Specific Port Write Function.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//! @param [in]  u8 *buffer         - buffer containing num_bytes of data
//! @param [in]  u32 num_bytes      - the number of bytes to write to the port
//! @param [out] u32 *bytes_read    - the number of bytes actually read from the device
//! @param [in]  u32 timeout_ms     - the read timeout
//
//! @retval MIP_USER_FUNCTION_ERROR  When there is a problem communicating with the port.\n
//! @retval MIP_USER_FUNCTION_OK     The read was successful.\n
//
//! @section NOTES
//!
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u16 mip_sdk_port_read(void* port_handle, u8* buffer, u32 num_bytes, u32* bytes_read, u32 timeout_ms) {
  HANDLE handle;
  DWORD local_bytes_read;

  // Set the bytes read to zero
  *bytes_read = 0;

  // Check for a valid port handle
  if (port_handle == NULL) return MIP_USER_FUNCTION_ERROR;

  handle = (HANDLE)(port_handle);

  // Call the windows read function
  if (ReadFile(handle, buffer, num_bytes, &local_bytes_read, NULL)) {
    *bytes_read = local_bytes_read;
    if (*bytes_read == num_bytes)
      return MIP_USER_FUNCTION_OK;
    else
      return MIP_USER_FUNCTION_ERROR;
  }

  return MIP_USER_FUNCTION_ERROR;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_port_read_count(void *port_handle)
//
//! @section DESCRIPTION
//! Target-Specific Function to Get the Number of Bytes Waiting on the Port.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Number of bytes waiting on the port,\n
//!           0, if there is an error.
//
//! @section NOTES
//!
//! None
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_port_read_count(void* port_handle) {
  COMSTAT com_status;
  DWORD errors;
  HANDLE handle;

  // Check for a valid port handle
  if (port_handle == NULL) return 0;

  handle = (HANDLE)(port_handle);

  // This function gets the current com status
  if (ClearCommError(handle, &errors, &com_status)) {
    return com_status.cbInQue;
  }

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
//
//! @fn
//! u32 mip_sdk_get_time_ms()
//
//! @section DESCRIPTION
//! Target-Specific Call to get the current time in milliseconds.
//
//! @section DETAILS
//!
//! @param [in]  void *port_handle  - target-specific port handle pointer (user needs to allocate memory for this)
//
//! @returns  Current time in milliseconds.
//
//! @section NOTES
//!
//!   1) This value should no roll-over in short periods of time (e.g. minutes)\n
//!   2) Most systems have a millisecond counter that rolls-over every 32 bits\n
//!      (e.g. 49.71 days roll-over period, with 1 millisecond LSB)\n
//!   3) An absolute reference is not required since this function is\n
//!      used for relative time-outs.
//!
//
/////////////////////////////////////////////////////////////////////////////

u32 mip_sdk_get_time_ms() {
  // GetTickCount is a windows function that returns the time
  // that has elapsed since system start-up (up to ~49 days)
  return GetTickCount();
}
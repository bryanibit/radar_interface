/**
 *  This file is a part of radar_interface.
 *
 *  Copyright (C) 2018 Juraj Persic, University of Zagreb Faculty of Electrical
 Engineering and Computing

 *  radar_interface is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TCP_INTERFACE_H
#define TCP_INTERFACE_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <cstdio>
#include <iostream>
#include <unistd.h>

enum return_statuses {
  OK = 0,
  INIT_FAILED = -1,
  BAD_PARAM = -2,
  SOCKET_ERROR = -3,
  SOCKET_CLOSED = -4,
  NO_MESSAGES_RECEIVED = -5,
  READ_FAILED = -6,
  WRITE_FAILED = -7,
  CLOSE_FAILED = -8,
  SOCKET_TIMEOUT = -9
};

class TCPInterface {
public:
  TCPInterface();
  ~TCPInterface();
  return_statuses open(const char *ip_address, const int &port);

  // Close the ethernet link
  return_statuses close();

  // Check on the status of the link
  bool is_open();

  // Read a message
  return_statuses
  read(unsigned char *msg, const size_t &buf_size, size_t &bytes_read,
       int timeout_ms = 0); // Optional timeout argument, in milliseconds
  return_statuses read_exactly(
      unsigned char *msg, const size_t &buf_size, const size_t &bytes_to_read,
      int timeout_ms = 0); // Optional timeout argument, in milliseconds

  // Send a message
  return_statuses write(unsigned char *msg, const size_t &msg_size);

  template <typename T>
  void parse_value(T *result, const unsigned char *msg,
                   const unsigned int offset, const unsigned int size) {
    T retVal = 0;

    for (unsigned int i = size; i > 0; i--) {
      retVal <<= 8;
      // Need to use -1 because array is 0-based
      // and offset is not.
      retVal |= msg[(offset - 1) + i];
    }
    *result = retVal;
  };

  template <typename T>
  void parse_array(T *array, const unsigned char *msg, unsigned int offset,
                   unsigned int element_size, unsigned int array_size) {
    T retVal = 0;
    for (int i = 0; i < array_size; i++) {
      for (int j = element_size; j > 0; j--) {
        retVal <<= 8;
        // Need to use -1 because array is 0-based
        // and offset is not.
        retVal |= msg[(offset - 1) + j];
      }
      array[i] = retVal;
    }
  }

  unsigned long parse_value_old(const unsigned char *msg, unsigned int offset,
                                unsigned int size);

  // unsigned long parse_array(const unsigned char *msg, unsigned long *array,
  //                           unsigned int offset, unsigned int element_size,
  //                           unsigned int array_size);

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::system::error_code error_;
  size_t bytes_read_;

  void timeout_handler(const boost::system::error_code &error);
  void read_handler(const boost::system::error_code &error, size_t bytes_read);
};

#endif
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

#include "radar_interface/tcp_interface.h"

using namespace std;
using boost::asio::ip::tcp;

// Default constructor.
TCPInterface::TCPInterface() : socket_(io_service_) {}

// Default destructor.
TCPInterface::~TCPInterface() {}

return_statuses TCPInterface::open(const char *ip_address, const int &port) {
  if (socket_.is_open())
    return OK;

  stringstream sPort;
  sPort << port;
  tcp::resolver res(io_service_);
  tcp::resolver::query query(tcp::v4(), ip_address, sPort.str());
  tcp::resolver::iterator it = res.resolve(query);
  boost::system::error_code ec;

  socket_.connect(*it, ec);

  if (ec.value() == boost::system::errc::success) {
    return OK;
  } else if (ec.value() == boost::asio::error::invalid_argument) {
    return BAD_PARAM;
  } else {
    close();
    return INIT_FAILED;
  }
}

return_statuses TCPInterface::close() {
  if (!socket_.is_open())
    return OK;

  boost::system::error_code ec;
  socket_.close(ec);

  if (ec.value() == boost::system::errc::success) {
    return OK;
  } else {
    return CLOSE_FAILED;
  }
}

bool TCPInterface::is_open() { return socket_.is_open(); }

void TCPInterface::timeout_handler(
    const boost::system::error_code &error) { // If the operation was not
                                              // aborted, store the bytes that
                                              // were read and set the read flag
  if (error != boost::asio::error::operation_aborted) {
    error_.assign(boost::system::errc::timed_out,
                  boost::system::system_category());
  }
}

void TCPInterface::read_handler(const boost::system::error_code &error,
                                size_t bytes_read) {
  bytes_read_ = bytes_read;
}

return_statuses TCPInterface::read(unsigned char *msg, const size_t &buf_size,
                                   size_t &bytes_read, int timeout_ms) {
  if (!socket_.is_open())
    return SOCKET_CLOSED;

  error_.assign(boost::system::errc::success, boost::system::system_category());

  boost::asio::deadline_timer timer(io_service_);
  // If requested timeout duration is set to 0 ms, don't set a deadline
  if (timeout_ms > 0) {
    timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    timer.async_wait(boost::bind(&TCPInterface::timeout_handler, this,
                                 boost::asio::placeholders::error));
  }

  boost::asio::async_read(
      socket_, boost::asio::buffer(msg, buf_size),
      boost::bind(&TCPInterface::read_handler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
  // Run until a handler is called
  while (io_service_.run_one()) {
    if (error_.value() == boost::system::errc::success) {
      timer.cancel();
      bytes_read = bytes_read_;
    } else if (error_.value() == boost::system::errc::timed_out) {
      socket_.cancel();
    }
  }
  // Reset the io service so that it is available for the next call to
  // TCPInterface::read
  io_service_.reset();

  if (error_.value() == boost::system::errc::success) {
    return OK;
  } else if (error_.value() == boost::system::errc::timed_out) {
    return SOCKET_TIMEOUT;
  } else {
    return READ_FAILED;
  }
}

return_statuses TCPInterface::read_exactly(unsigned char *msg,
                                           const size_t &buf_size,
                                           const size_t &bytes_to_read,
                                           int timeout_ms) {
  if (!socket_.is_open())
    return SOCKET_CLOSED;

  error_.assign(boost::system::errc::success, boost::system::system_category());

  boost::asio::deadline_timer timer(io_service_);
  // If requested timeout duration is set to 0 ms, don't set a deadline
  if (timeout_ms > 0) {
    timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    timer.async_wait(boost::bind(&TCPInterface::timeout_handler, this,
                                 boost::asio::placeholders::error));
  }

  boost::asio::async_read(
      socket_, boost::asio::buffer(msg, buf_size),
      boost::asio::transfer_exactly(bytes_to_read),
      boost::bind(&TCPInterface::read_handler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
  // Run until a handler is called
  while (io_service_.run_one()) {
    if (error_.value() == boost::system::errc::success) {
      timer.cancel();
    } else if (error_.value() == boost::system::errc::timed_out) {
      socket_.cancel();
    }
  }
  // Reset the io service so that it is available for the next call to
  // TCPInterface::read_exactly
  io_service_.reset();

  if (error_.value() == boost::system::errc::success) {
    return OK;
  } else if (error_.value() == boost::system::errc::timed_out) {
    return SOCKET_TIMEOUT;
  } else {
    return READ_FAILED;
  }
}

return_statuses TCPInterface::write(unsigned char *msg,
                                    const size_t &msg_size) {
  if (!socket_.is_open())
    return SOCKET_CLOSED;

  boost::system::error_code ec;
  boost::asio::write(socket_, boost::asio::buffer(msg, msg_size), ec);

  if (ec.value() == boost::system::errc::success) {
    return OK;
  } else {
    return WRITE_FAILED;
  }
}

// void TCPInterface::parse_value(T *result, const unsigned char *msg,
//                                const unsigned int offset,
//                                const unsigned int size) {
//   T retVal = 0;

//   for (unsigned int i = size; i > 0; i--) {
//     retVal <<= 8;
//     // Need to use -1 because array is 0-based
//     // and offset is not.
//     retVal |= msg[(offset - 1) + i];
//   }
//   *result = retVal;
// }

unsigned long TCPInterface::parse_value_old(const unsigned char *msg,
                                            const unsigned int offset,
                                            const unsigned int size) {
  unsigned long retVal = 0;

  for (unsigned int i = size; i > 0; i--) {
    retVal <<= 8;
    // Need to use -1 because array is 0-based
    // and offset is not.
    retVal |= msg[(offset - 1) + i];
  }

  return retVal;
}

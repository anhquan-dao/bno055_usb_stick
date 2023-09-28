#ifndef BNO055_USB_STICK_BNO055_USB_STICK_HPP
#define BNO055_USB_STICK_BNO055_USB_STICK_HPP

#include <algorithm>
#include <deque>
#include <sstream>
#include <string>
#include <iomanip>

// #include <ros/console.h>
// #include <ros/duration.h>
// #include <ros/names.h>
// #include <ros/param.h>

#include <rclcpp/rclcpp.hpp>

#include <bno055_usb_stick/constants.hpp>
#include <bno055_usb_stick/decoder.hpp>
#include <bno055_usb_stick/match_conditions.hpp>
#include <bno055_usb_stick/msg/output.hpp>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/cstdint.hpp>
#include <boost/function.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>


namespace bno055_usb_stick {

class BNO055USBStick {
public:
  typedef boost::function< void(const bno055_usb_stick::msg::Output &) > Callback;

public:
  BNO055USBStick(boost::asio::io_service &asio_service, const Callback &callback,
                 const std::shared_ptr<rclcpp::Node> node)
      : callback_(callback) {

    node_ = node;
    port_ = node_->get_parameter("port").as_string();
    mode_ = node_->get_parameter("mode").as_string();

    double timeout = node_->get_parameter("timeout").as_double();
    timeout_ = std::make_shared<rclcpp::Duration>(timeout*1.0e+09);

    decoder_ = std::make_shared<Decoder>(node_);
    serial_ = std::make_shared<boost::asio::serial_port>(asio_service);
    timer_ = std::make_shared<boost::asio::deadline_timer>(asio_service);
    
    start();
  }

  virtual ~BNO055USBStick() { stop(); }

private:
  void start() {
    // stop the previous operaiton if there
    stop();

    // setup the serial port
    try {
      serial_->open(port_);

      typedef boost::asio::serial_port Serial;
      serial_->set_option(Serial::baud_rate(115200));
      serial_->set_option(Serial::flow_control(Serial::flow_control::none));
      serial_->set_option(Serial::parity(Serial::parity::none));
      serial_->set_option(Serial::stop_bits(Serial::stop_bits::one));
      serial_->set_option(Serial::character_size(8));
    } catch (const boost::system::system_error &error) {
      // retry if something is wrong
      RCLCPP_ERROR(node_->get_logger(), "start: %s", error.what());
      restart();
      return;
    }

    // pack commands
    commands_.clear();
    if (mode_ == "ndof") {
      for (const boost::uint8_t **command = Constants::toNDOFCommands(); *command; ++command) {
        commands_.push_back(*command);
      }
    } else if (mode_ == "imu") {
      for (const boost::uint8_t **command = Constants::toIMUCommands(); *command; ++command) {
        commands_.push_back(*command);
      }
    } else {
      RCLCPP_WARN(node_->get_logger(),"Unknown mode \" %s \" was given. Will use the default mode \"ndof\" instead.", mode_ );
      for (const boost::uint8_t **command = Constants::toNDOFCommands(); *command; ++command) {
        commands_.push_back(*command);
      }
    }
    for (const boost::uint8_t **command = Constants::startStreamCommands(); *command; ++command) {
      commands_.push_back(*command);
    }

    // trigger send packed commands
    startSendCommand();
  }

  void startSendCommand() {
    if (commands_.empty()) {
      RCLCPP_ERROR(node_->get_logger(),"startSendCommand: No command in the queue");
      restart();
      return;
    }

    // trigger send the top command in the queue
    const boost::uint8_t *command(commands_.front());
    boost::asio::async_write(*serial_,
                             boost::asio::buffer(command, Constants::getCommandLength(command)),
                             boost::bind(&BNO055USBStick::handleSendCommand, this, _1, _2));

    // schedule restarting in case of timeout
    startWaitDeadline(&BNO055USBStick::restart);
  }

  void handleSendCommand(const boost::system::error_code &error, const std::size_t bytes) {
    // cancel the timeout action
    cancelWaitDeadline();

    if (error) {
      RCLCPP_ERROR(node_->get_logger(),"handleSendCommand: %s", error.message());
      restart();
      return;
    }

    // pop the top command from the queue
    // dumpWritten("handleSendCommand: written: ", bytes);
    commands_.pop_front();

    // trigger wait the response for the command
    startWaitResponse();
  }

  void startWaitResponse() {
    // trigger read a responce
    boost::asio::async_read_until(*serial_, buffer_, ResponseCondition(),
                                  boost::bind(&BNO055USBStick::handleWaitResponse, this, _1, _2));

    // schedule restarting in case of timeout
    startWaitDeadline(&BNO055USBStick::restart);
  }

  void handleWaitResponse(const boost::system::error_code &error, const std::size_t bytes) {
    // cancel the timeout action
    cancelWaitDeadline();

    if (error) {
      RCLCPP_ERROR(node_->get_logger(),"handleWaitResponse: %s ", error.message());
      restart();
      return;
    }

    // clear the read response (cannot parse it because the protocol is unknown...)
    // dumpRead("handleWaitResponse: read: ", bytes);
    buffer_.consume(bytes);

    // trigger send the next command, or wait data stream
    if (!commands_.empty()) {
      startSendCommand();
    } else {
      startWaitData();
    }
  }

  void startWaitData() {
    // trigger read a data
    boost::asio::async_read_until(*serial_, buffer_, DataCondition(),
                                  boost::bind(&BNO055USBStick::handleWaitData, this, _1, _2));

    // schedule restarting in case of timeout
    startWaitDeadline(&BNO055USBStick::restart);
  }

  void handleWaitData(const boost::system::error_code &error, const std::size_t bytes) {
    // cancel the timeout action
    cancelWaitDeadline();

    if (error) {
      RCLCPP_ERROR(node_->get_logger(),"handleWaitData: %s", error.message());
      restart();
      return;
    }

    // decode the received data and execute the user callback
    // dumpRead("handleWaitData: read: ", bytes);
    if (callback_) {
      const boost::uint8_t *data_end(
          boost::asio::buffer_cast< const boost::uint8_t * >(buffer_.data()) + bytes);
      const boost::uint8_t *data_begin(data_end - Constants::DAT_LEN);
      const bno055_usb_stick::msg::Output output(decoder_->decode(data_begin));
      callback_(output);
    }

    // clear the parsed data
    buffer_.consume(bytes);

    // trigger wait the next data
    startWaitData();
  }

  void stop() {
    // finalize the serial port
    try {
      if (serial_->is_open()) {
        serial_->close();
      }
    } catch (const boost::system::system_error &error) {
      // just print the error because can do nothing further
      RCLCPP_ERROR(node_->get_logger(),"stop: %s", error.what());
    }
  }

  void restart() {
    // schedule start a new operation
    startWaitDeadline(&BNO055USBStick::start);
  }

  void startWaitDeadline(void (BNO055USBStick::*handler)()) {
    boost::posix_time::time_duration boost_timeout;
    #if defined(BOOST_DATE_TIME_HAS_NANOSECONDS)
      boost_timeout = boost::posix_time::nanoseconds(timeout_->nanoseconds());
    #else
      boost_timeout = boost::posix_time::microseconds(timeout_->nanoseconds()/1000);
    #endif
    
    timer_->expires_from_now(boost_timeout);
    timer_->async_wait(boost::bind(&BNO055USBStick::handleWaitDeadline, this, _1, handler));
  }

  void cancelWaitDeadline() { timer_->cancel(); }

  void handleWaitDeadline(const boost::system::error_code &error,
                          void (BNO055USBStick::*handler)()) {
    // make sure this callback is called by a deadline expiration
    if (error == boost::asio::error::operation_aborted) {
      // RCLCPP_INFO(node_->get_logger(),"handleWaitDeadline: the deadline disabled");
      return;
    } else if (error) {
      RCLCPP_ERROR(node_->get_logger(),"handleWaitDeadline: %s", error.message());
      return;
    }

    // try to cancel all operations on the serial port
    try {
      if (serial_->is_open()) {
        serial_->cancel();
      }
    } catch (const boost::system::system_error &error_on_cancel) {
      RCLCPP_ERROR(node_->get_logger(),"handleWaitDeadline: %s", error_on_cancel.what());
    }

    // execute the given handler
    if (handler) {
      (this->*handler)();
    }
  }

  void dumpWritten(const std::string &prefix, const std::size_t bytes) {
    std::ostringstream oss;
    const boost::uint8_t *begin(commands_.front());
    const boost::uint8_t *end(begin + bytes);
    for (const boost::uint8_t *c = begin; c != end; ++c) {
      oss << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(*c) << " ";
    }
    RCLCPP_INFO(node_->get_logger(),"%s%s", prefix, oss.str());
  }

  void dumpRead(const std::string &prefix, const std::size_t bytes) {
    std::ostringstream oss;
    const boost::uint8_t *begin(boost::asio::buffer_cast< const boost::uint8_t * >(buffer_.data()));
    const boost::uint8_t *end(begin + bytes);
    for (const boost::uint8_t *c = begin; c != end; ++c) {
      oss << "0x" << std::setw(2) << std::setfill('0') << std::hex << int(*c) << " ";
    }
    RCLCPP_INFO(node_->get_logger(),"%s%s", prefix, oss.str());
  }

private:
  // parameters
  std::string port_;
  std::shared_ptr<rclcpp::Duration> timeout_;
  std::string mode_;

  // buffers
  std::deque< const boost::uint8_t * > commands_;
  boost::asio::streambuf buffer_;

  // async objects
  std::shared_ptr<boost::asio::serial_port> serial_;
  std::shared_ptr<boost::asio::deadline_timer> timer_;

  // callback given by the user
  const Callback callback_;

  // orientation and sensor decoder
  std::shared_ptr<Decoder> decoder_;

  std::shared_ptr<rclcpp::Node> node_;
};
}

#endif // BNO055_USB_STICK_BNO055_USB_STICK_HPP
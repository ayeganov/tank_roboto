#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/system/error_code.hpp>
#include <iostream>
#include <unistd.h>

using namespace boost::asio;


int main()
{
  io_service ioservice;

  posix::stream_descriptor stream{ioservice, STDOUT_FILENO};
  streambuf buf;
  auto handler = [&](const boost::system::error_code&, std::size_t) {
      std::istream is(&buf);
      std::string out;
      is >> out;
    std::cout << "Got string: " << out << std::endl;
  };


  async_read(stream, buf, handler);

  ioservice.run();
}

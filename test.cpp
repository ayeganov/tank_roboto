#include <cstdio>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <termios.h>
#include <unistd.h>


class Input : public boost::enable_shared_from_this<Input>
{
public:
    typedef boost::shared_ptr<Input> Ptr;

public:
    static void create(
            boost::asio::io_service& io_service
            )
    {
        Ptr input(
                new Input( io_service )
                );
        input->read();
    }

private:
    explicit Input(
            boost::asio::io_service& io_service
         ) :
        _input( io_service )
    {
        _input.assign( STDIN_FILENO );
    }

    void read()
    {
        boost::asio::async_read(
                _input,
                boost::asio::buffer( &_command, sizeof(_command) ),
                boost::bind(
                    &Input::read_handler,
                    shared_from_this(),
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred
                    )
                );
    }

    void read_handler(
            const boost::system::error_code& error,
            const size_t bytes_transferred
            )
    {
        if ( error ) {
            std::cerr << "read error: " << boost::system::system_error(error).what() << std::endl;
            return;
        }

        if ( _command != '\n' ) {
            std::cout << "command: " << _command << std::endl;
        }

        this->read();
    }

private:
    boost::asio::posix::stream_descriptor _input;
    char _command;
};


int main()
{
  boost::asio::io_service io;

  int count = 0;

  struct termios term;
  if(tcgetattr(STDIN_FILENO, &term))
  {
      std::cerr << "tcgetttr failed\n";
      exit(-1);
  }
  term.c_lflag &= ~ICANON;
  term.c_lflag &= ~ECHO;
  term.c_cc[VMIN] = 1;

  if(tcsetattr(STDIN_FILENO, TCSANOW, &term))
  {
      std::cerr << "tcsetattr failed\n";
      exit(-1);
  }

  Input::create( io);

  io.run();

  std::cout << "Final count is " << count << "\n";

  return 0;
}

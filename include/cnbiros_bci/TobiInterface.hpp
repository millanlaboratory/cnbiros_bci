#ifndef CNBIROS_BCI_TOBIINTERFACE_HPP
#define CNBIROS_BCI_TOBIINTERFACE_HPP

#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>

namespace cnbiros {
	namespace bci {

class TobiInterface {
	public:
		TobiInterface(void);
		virtual ~TobiInterface(void);

		virtual bool configure(void);

		bool Connect(void);
		void Disconnect(void);

		virtual bool Attach(void) {};
		virtual bool Detach(void) {};

		virtual void Run(void) = 0;

		bool IsConnected(void);
		bool IsConfigured(void);
		bool IsAttached(void);

		int GetMode(void);
		std::string GetModeName(void);

		bool SetMode(const std::string& mode);
		bool SetMode(int mode);

	public:
		static const int Undefined	= -1;
		static const int SetOnly	=  0;
		static const int GetOnly	=  1;
		static const int SetGet		=  2;

	protected:
		ros::NodeHandle	nh_;
		ros::NodeHandle	p_nh_;

		std::string		pipe_;
		std::string		nmode_;
	
	private:
		int		mode_;
		bool	do_reconnect_;
		float	rtime_;
		bool	is_configured_;
		bool	is_attached_;
		std::string	loop_ip_;

};

	}
}

#endif

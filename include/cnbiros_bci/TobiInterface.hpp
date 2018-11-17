#ifndef CNBIROS_BCI_TOBIINTERFACE_HPP
#define CNBIROS_BCI_TOBIINTERFACE_HPP

#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>

namespace cnbiros {
	namespace bci {

class TobiInterface {
	public:
		TobiInterface(void);
		TobiInterface(const std::string& loopip, float rtime);
		virtual ~TobiInterface(void);

		bool Connect(void);
		void Disconnect(void);
		bool IsConnected(void);

		int GetMode(void);
		std::string GetModeName(void);

		bool SetMode(const std::string& mode);
		bool SetMode(int mode);
		
		virtual bool Run(void) = 0;

	public:
		static const int Undefined	= -1;
		static const int SetOnly	=  0;
		static const int GetOnly	=  1;
		static const int SetGet		=  2;

	protected:
		float			rtime_;
		int				mode_;
		std::string		nmode_;
		std::string		loop_ip_;

};

	}
}

#endif

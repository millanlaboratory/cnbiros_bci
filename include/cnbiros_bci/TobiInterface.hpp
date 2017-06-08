#ifndef CNBIROS_BCI_TOBIINTERFACE_HPP
#define CNBIROS_BCI_TOBIINTERFACE_HPP

#include <ros/ros.h>
#include <cnbiloop/ClLoop.hpp>

namespace cnbiros {
	namespace bci {

class TobiInterface {
	public:
		TobiInterface(CcAddress address = "127.0.0.1:8123");
		virtual ~TobiInterface(void);

		void Connect(void);
		void Disconnect(void);
		bool IsConnected(void);

		std::string GetAddress(void);

	private:
		CcAddress address_;

};

	}
}


#endif

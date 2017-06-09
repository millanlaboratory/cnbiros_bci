#ifndef CNBIROS_BCI_TICTOOLS_HPP
#define CNBIROS_BCI_TICTOOLS_HPP

#include <vector>

#include <tobiic/ICMessage.hpp>
#include "cnbiros_bci/TicClass.h"
#include "cnbiros_bci/TicClassifier.h"
#include "cnbiros_bci/TicMessage.h"

namespace cnbiros {
	namespace bci {

class TicTools {
	
	public:
		TicTools(void);
		~TicTools(void);

		ICMessage GetMessage(const cnbiros_bci::TicMessage& icmros);
		
		std::vector<cnbiros_bci::TicMessage> GetMessage(const ICMessage& iccnbi, const std::string& pipe);

	private:
		std::vector<ICClass*> 	icclasses_;
		ICClassifier* 		 	icclassifier_;

};

	}
}

#endif

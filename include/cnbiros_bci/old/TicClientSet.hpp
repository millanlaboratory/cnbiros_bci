#ifndef CNBIROS_BCI_TICCLIENTSET_HPP
#define CNBIROS_BCI_TICCLIENTSET_HPP

#include <memory>
#include <unordered_map>
#include <cnbiloop/ClTobiIc.hpp>

namespace cnbiros {
	namespace bci {

// Edited by L.Tonin  <luca.tonin@epfl.ch> on 12/07/17 10:59:55
// Shared pointer version. Not used because the bug in the ClTobiIc destructor
typedef std::unordered_map<std::string, std::shared_ptr<ClTobiIc>> TicClientMap;
typedef std::unordered_map<std::string, std::shared_ptr<ClTobiIc>>::iterator TicClientMapIt;
typedef std::unordered_map<std::string, std::shared_ptr<ClTobiIc>>::const_iterator TicClientMapConstIt;

//typedef std::unordered_map<std::string, ClTobiIc*> TicClientMap;
//typedef std::unordered_map<std::string, ClTobiIc*>::iterator TicClientMapIt;
//typedef std::unordered_map<std::string, ClTobiIc*>::const_iterator TicClientMapConstIt;

class TicClientSet {

	public:
		TicClientSet(void);
		~TicClientSet(void);
	
		TicClientMapIt Find(const std::string& pipe);
		bool Exist(const std::string& pipe);
		bool Remove(const std::string& pipe);
		void Erase(void);
		
		bool Get(const std::string& pipe, std::shared_ptr<ClTobiIc>& tic);

		bool Add(const std::string& pipe, unsigned int mode);
		void Dump(void);

		TicClientMapIt Begin(void);
		TicClientMapIt End(void);
		TicClientMapConstIt Begin(void) const;
		TicClientMapConstIt End(void) const; 

	private:
		TicClientMap 	ticclset_;
};

	}
}


#endif

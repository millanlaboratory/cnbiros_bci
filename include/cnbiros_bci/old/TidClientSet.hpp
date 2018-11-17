#ifndef CNBIROS_BCI_TIDCLIENTSET_HPP
#define CNBIROS_BCI_TIDCLIENTSET_HPP

#include <memory>
#include <unordered_map>
#include <cnbiloop/ClTobiId.hpp>

namespace cnbiros {
	namespace bci {

// Edited by L.Tonin  <luca.tonin@epfl.ch> on 12/07/17 10:59:55
// Shared pointer version. Not used because the bug in the ClTobiIc destructor
typedef std::unordered_map<std::string, std::shared_ptr<ClTobiId>> TidClientMap;
typedef std::unordered_map<std::string, std::shared_ptr<ClTobiId>>::iterator TidClientMapIt;
typedef std::unordered_map<std::string, std::shared_ptr<ClTobiId>>::const_iterator TidClientMapConstIt;

//typedef std::unordered_map<std::string, ClTobiId*> TidClientMap;
//typedef std::unordered_map<std::string, ClTobiId*>::iterator TidClientMapIt;
//typedef std::unordered_map<std::string, ClTobiId*>::const_iterator TidClientMapConstIt;

class TidClientSet {

	public:
		TidClientSet(void);
		~TidClientSet(void);
	
		TidClientMapIt Find(const std::string& pipe);
		bool Exist(const std::string& pipe);
		bool Remove(const std::string& pipe);
		void Erase(void);
		
		bool Get(const std::string& pipe, std::shared_ptr<ClTobiId>& tid);

		bool Add(const std::string& pipe, unsigned int mode);

		TidClientMapIt Begin(void);
		TidClientMapIt End(void);
		TidClientMapConstIt Begin(void) const;
		TidClientMapConstIt End(void) const; 

	private:
		TidClientMap 	tidclset_;
};

	}
}


#endif

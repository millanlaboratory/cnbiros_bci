#ifndef CNBIROS_BCI_TIDCLIENTSET_HPP
#define CNBIROS_BCI_TIDCLIENTSET_HPP

#include <unordered_map>
#include <cnbiloop/ClTobiId.hpp>

namespace cnbiros {
	namespace bci {

typedef std::unordered_map<std::string, ClTobiId*> TidClientMap;
typedef std::unordered_map<std::string, ClTobiId*>::iterator TidClientMapIt;
typedef std::unordered_map<std::string, ClTobiId*>::const_iterator TidClientMapConstIt;

class TidClientSet {

	public:
		TidClientSet(void);
		~TidClientSet(void);
	
		TidClientMapIt Find(const std::string& pipe);
		bool Exist(const std::string& pipe);
		bool Remove(const std::string& pipe);
		void Erase(void);
		
		bool Get(const std::string& pipe, ClTobiId*& tid);

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

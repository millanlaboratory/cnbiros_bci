#ifndef CNBIROS_BCI_TIDCLIENTSET_CPP
#define CNBIROS_BCI_TIDCLIENTSET_CPP

#include "cnbiros_bci/TidClientSet.hpp"

namespace cnbiros {
	namespace bci {

TidClientSet::TidClientSet(void) {};
TidClientSet::~TidClientSet(void) {
	this->Erase();
};

bool TidClientSet::Add(const std::string& pipe, unsigned int mode) {
	//bool retcod = true;
	//this->tidclset_[pipe] = new ClTobiId(mode);
	//return retcod;
	auto result = this->tidclset_.emplace(pipe, std::make_shared<ClTobiId>(mode));
	return result.second;
}

TidClientMapIt TidClientSet::Find(const std::string& pipe) {
	return this->tidclset_.find(pipe);
}

bool TidClientSet::Exist(const std::string& pipe) {

	bool retcod = false;

	if( this->Find(pipe) != this->tidclset_.end() )
		retcod = true;

	return retcod;
}

bool TidClientSet::Remove(const std::string& pipe) {

	bool retcod = false;
	TidClientMapIt it = this->Find(pipe);

	if( it != this->tidclset_.end() ) {
		it->second->Detach();
		this->tidclset_.erase(it);
		//delete it->second;
		retcod = true;
	}

	return retcod;
}

void TidClientSet::Erase(void) {

	for(auto it=this->tidclset_.begin(); it!=tidclset_.end(); ++it) {
		if(it->second->IsAttached())
			it->second->Detach();
		//delete it->second;
	}

	this->tidclset_.clear();
}

bool TidClientSet::Get(const std::string& pipe, std::shared_ptr<ClTobiId>& tid) {

	bool retcod = false;

	TidClientMapIt it = this->Find(pipe);
	if( it != this->tidclset_.end() ) {
		tid = (it->second);
		retcod = true;
	}

	return retcod;
}

TidClientMapIt TidClientSet::Begin(void) {
	return this->tidclset_.begin();
}

TidClientMapIt TidClientSet::End(void) {
	return this->tidclset_.end();
}

TidClientMapConstIt TidClientSet::Begin(void) const {
	return this->tidclset_.begin();
}

TidClientMapConstIt TidClientSet::End(void) const {
	return this->tidclset_.end();
}
	}
}

#endif

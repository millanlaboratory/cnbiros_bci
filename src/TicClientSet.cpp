#ifndef CNBIROS_BCI_TICCLIENTSET_CPP
#define CNBIROS_BCI_TICCLIENTSET_CPP

#include "cnbiros_bci/TicClientSet.hpp"

namespace cnbiros {
	namespace bci {

TicClientSet::TicClientSet(void) {};
TicClientSet::~TicClientSet(void) {
	this->Erase();
};

bool TicClientSet::Add(const std::string& pipe, unsigned int mode) {
	bool retcod = true;
	this->ticclset_[pipe] = new ClTobiIc(mode);
	return retcod;
}

TicClientMapIt TicClientSet::Find(const std::string& pipe) {
	return this->ticclset_.find(pipe);
}

bool TicClientSet::Exist(const std::string& pipe) {

	bool retcod = false;

	if( this->Find(pipe) != this->ticclset_.end() )
		retcod = true;

	return retcod;
}

bool TicClientSet::Remove(const std::string& pipe) {

	bool retcod = false;
	TicClientMapIt it = this->Find(pipe);

	if( it != this->ticclset_.end() ) {
		delete it->second;
		this->ticclset_.erase(it);
		retcod = true;
	}

	return retcod;
}

void TicClientSet::Erase(void) {

	for(auto it=this->ticclset_.begin(); it!=ticclset_.end(); ++it) {
		if(it->second->IsAttached())
			it->second->Detach();
		delete it->second;
	}

	this->ticclset_.clear();
}

bool TicClientSet::Get(const std::string& pipe, ClTobiIc*& tic) {

	bool retcod = false;

	TicClientMapIt it = this->Find(pipe);
	if( it != this->ticclset_.end() ) {
		tic = (it->second);
		retcod = true;
	}

	return retcod;
}

TicClientMapIt TicClientSet::Begin(void) {
	return this->ticclset_.begin();
}

TicClientMapIt TicClientSet::End(void) {
	return this->ticclset_.end();
}

TicClientMapConstIt TicClientSet::Begin(void) const {
	return this->ticclset_.begin();
}

TicClientMapConstIt TicClientSet::End(void) const {
	return this->ticclset_.end();
}
	}
}

#endif

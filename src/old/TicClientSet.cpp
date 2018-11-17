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
	auto result = this->ticclset_.emplace(pipe, std::make_shared<ClTobiIc>(mode));
	return result.second;
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
		it->second->Detach();
		this->ticclset_.erase(it);
		retcod = true;
	}

	return retcod;
}

void TicClientSet::Dump(void) {
	printf("Dump TiCClientSet:\n");
	for(auto it=this->ticclset_.begin(); it!=ticclset_.end(); ++it) {
		printf("|- %s : %p\n", it->first.c_str(), it->second.get());
	}
}

void TicClientSet::Erase(void) {

	for(auto it=this->ticclset_.begin(); it!=ticclset_.end(); ++it) {
		if(it->second->IsAttached())
			it->second->Detach();
	}

	this->ticclset_.clear();
}

bool TicClientSet::Get(const std::string& pipe, std::shared_ptr<ClTobiIc>& tic) {

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

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
	//this->ticclset_[pipe] = new ClTobiIc(mode);
	//this->ticclset_.emplace(pipe, new ClTobiIc(mode));
	// Edited by L.Tonin  <luca.tonin@epfl.ch> on 12/07/17 10:58:49
	// Shared pointer version. Not used because the bug in the destructor of
	// ClTobiIc
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
		// Edited by L.Tonin  <luca.tonin@epfl.ch> on 12/07/17 10:56:54	
		// Bug in ClTobiIc destructor. Lead to segmentation fault
		it->second->Detach();
		this->ticclset_.erase(it);
		//delete it->second;
		retcod = true;
	}

	return retcod;
}

void TicClientSet::Dump(void) {
	printf("Dump TiCClientSet:\n");
	for(auto it=this->ticclset_.begin(); it!=ticclset_.end(); ++it) {
		printf("|- %s : %p\n", it->first.c_str(), it->second);
	}
}

void TicClientSet::Erase(void) {

	for(auto it=this->ticclset_.begin(); it!=ticclset_.end(); ++it) {
		if(it->second->IsAttached())
			it->second->Detach();
		// Edited by L.Tonin  <luca.tonin@epfl.ch> on 12/07/17 10:56:54	
		// Bug in ClTobiIc destructor. Lead to segmentation fault
		//delete it->second; 
	}

	this->ticclset_.clear();
}

bool TicClientSet::Get(const std::string& pipe, std::shared_ptr<ClTobiIc>& tic) {

	bool retcod = false;

	TicClientMapIt it = this->Find(pipe);
	if( it != this->ticclset_.end() ) {
		// Edited by L.Tonin  <luca.tonin@epfl.ch> on 12/07/17 10:59:55
		// Shared pointer version. Not used because the bug in the ClTobiIc destructor
		//tic = it->second.get();
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

#ifndef CNBIROS_BCI_TIDTOOLS_HPP
#define CNBIROS_BCI_TIDTOOLS_HPP

#include <ros/ros.h>
#include <tobicore/TCException.hpp>
#include <tobiid/IDMessage.hpp>
#include "cnbiros_tobi_msgs/TidMessage.h"

namespace cnbiros {
	namespace bci {

//* TidTools class
/** \brief Support class to manage conversion between IDMessage and
 * cnbiros_bci::TidMessage.
 *
 * \par General description:
 * TidTools is a support class to handle in an easy way the conversion
 * between IDMessage (from tobiid library) and cnbiros_bci::TidMessage (in ROS
 * ecosystem). The class provides static methods to be used without
 * instantiation.
 *
 *	\todo 
 *	- Add new functionalities that replicate the IDMessage methods?
 *
 * \sa TiDProxy
 */
class TidTools {
	
	public:
		//! \brief Constructor
		TidTools(void);
		
		//! \brief Destructor
		~TidTools(void);

		/*! \brief Method to get the ID message in IDMessage (Tobi) format
		 *
		 * This methods convert the IDMessage (CNBI loop ecosystem)	into a
		 * cnbiros_bci::TidMessage
		 *
		 * \param[out] Conversion result
		 *
		 */
		static bool ToTobi(const cnbiros_tobi_msgs::TidMessage& mros, IDMessage& mtobi);
		
		/*! \brief Method to get the ID message in cnbiros_bci::TidMessage
		 * format
		 *
		 * This methods convert the cnbiros_bci::TidMessage into IDMessage (CNBI
		 * loop ecosystem)		 
		 * 
		 * \param[out]	Conversion result
		 *
		 */
		static bool ToRos(const IDMessage& mtobi, const std::string& pipe, cnbiros_tobi_msgs::TidMessage& mros);


};

	}
}

#endif

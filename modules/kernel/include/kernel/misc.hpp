/** \file misc.hpp Defines usefull functions
 *
 * \ingroup kernel
 */

#ifndef KERNEL_MISC_HPP
#define KERNEL_MISC_HPP

#include <vector>

namespace jafar {
namespace kernel {

	/**
	These functions erases a vector element without preserving order of elements,
	but with complexity O(1) instead of O(n)
	*/
	template <typename T>
	static inline void fastErase(std::vector<T> &v, typename std::vector<T>::iterator pos)
	{
		if (v.size() <= 1)
			v.resize(0);
		else
		{
			*pos = v.back();
			v.pop_back();
		}
	}
	
	template <typename T>
	static inline void fastErase(std::vector<T> &v, typename std::vector<T>::size_type index)
	{
		if (v.size() <= 1)
			v.resize(0);
		else
		{
			v.at(index) = v.back();
			v.pop_back();
		}
	} 

}}

#endif
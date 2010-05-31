/* -----------------------------------------------------------------------------
 * See the LICENSE file for information on copyright, usage and redistribution
 * of SWIG, and the README file for authors - http://www.swig.org/release.html.
 *
 * rubytracking.swg
 *
 * This file contains support for tracking mappings from 
 * Ruby objects to C++ objects.  This functionality is needed
 * to implement mark functions for Ruby's mark and sweep
 * garbage collector.
 * ----------------------------------------------------------------------------- */

#ifdef __cplusplus
// extern "C" {
#endif
  
#include <map>

/* Ruby 1.8 actually assumes the first case. */
#if SIZEOF_VOIDP == SIZEOF_LONG
#  define SWIG2NUM(v) LONG2NUM((unsigned long)v)
#  define NUM2SWIG(x) (unsigned long)NUM2LONG(x)
#elif SIZEOF_VOIDP == SIZEOF_LONG_LONG
#  define SWIG2NUM(v) LL2NUM((unsigned long long)v)
#  define NUM2SWIG(x) (unsigned long long)NUM2LL(x)
#else
#  error sizeof(void*) is not the same as long or long long
#endif


/* Global Ruby hash table to store Trackings from C/C++
   structs to Ruby Objects. 
*/
extern std::map<void*, VALUE>* swig_ruby_trackings = 0;

/* Setup a Ruby hash table to store Trackings */
SWIGRUNTIME void SWIG_RubyInitializeTrackings(void) {
  /* Create a ruby hash table to store Trackings from C++ 
     objects to Ruby objects. */

  /* Try to see if some other .so has already created a 
     tracking hash table, which we keep hidden in an instance var
     in the SWIG module.
     This is done to allow multiple DSOs to share the same
     tracking table.
  */

  /* No, it hasn't.  Create one ourselves */ 
  if ( !swig_ruby_trackings )
  {
      swig_ruby_trackings = new std::map<void*, VALUE>();
  }
}

/* Add a Tracking from a C/C++ struct to a Ruby object */
SWIGRUNTIME void SWIG_RubyAddTracking(void* ptr, VALUE object) {
  /* In a Ruby hash table we store the pointer and
     the associated Ruby object.  The trick here is
     that we cannot store the Ruby object directly - if
     we do then it cannot be garbage collected.  So
     instead we typecast it as a unsigned long and
     convert it to a Ruby number object.*/

  /* Store the mapping to the global hash table. */
  (*swig_ruby_trackings)[ ptr ] = object;
}

/* Get the Ruby object that owns the specified C/C++ struct */
SWIGRUNTIME VALUE SWIG_RubyInstanceFor(void* ptr) {
  /* Now lookup the value stored in the global hash table */

  std::map<void*, VALUE>::iterator it = swig_ruby_trackings->find( ptr );
  if ( it == swig_ruby_trackings->end()) {
    return Qnil;
  }
  return it->second;
}

/* Remove a Tracking from a C/C++ struct to a Ruby object.  It
   is very important to remove objects once they are destroyed
   since the same memory address may be reused later to create
   a new object. */
SWIGRUNTIME void SWIG_RubyRemoveTracking(void* ptr) {
  /* Delete the object from the hash table by calling Ruby's
     do this we need to call the Hash.delete method.*/
  swig_ruby_trackings->erase( ptr );
}

/* This is a helper method that unlinks a Ruby object from its
   underlying C++ object.  This is needed if the lifetime of the
   Ruby object is longer than the C++ object */
SWIGRUNTIME void SWIG_RubyUnlinkObjects(void* ptr) {
  VALUE object = SWIG_RubyInstanceFor(ptr);

  if (object != Qnil) {
    DATA_PTR(object) = 0;
  }
}


#ifdef __cplusplus
// }
#endif

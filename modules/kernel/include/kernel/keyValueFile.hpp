/* $Id$ */

#ifndef KERNEL_KEY_VALUE_FILE_HPP
#define KERNEL_KEY_VALUE_FILE_HPP

#include <string>
#include <sstream>
#include <map>

#include "kernel/kernelException.hpp"

namespace jafar {
  namespace kernel {

    // forward declaration
    class KeyValueFileLoad;
    class KeyValueFileSave;

    /** Read a file containing simple key value statements.
     *
     * \todo add support for a write function
     *
     * \ingroup kernel
     */
    class KeyValueFile {

    private:

      /// Key value separators, default to ":".
      std::string keyValueSeparator;
      char commentPrefix;

      typedef std::map<std::string, std::string> KeyValueMap;

      KeyValueMap keyValue;

    public:

      KeyValueFile(std::string const& keyValueSeparator_ = ":", char commentPrefix_ = '#');

      /// Parse file \a filename for key:value pairs.
      void readFile(std::string const& filename);

      /// get the \a value of item \a key.
      template<class T>
      void getItem(std::string const& key, T& value) const {

	KeyValueMap::const_iterator it = keyValue.find(key);

	JFR_PRED_ERROR(it != keyValue.end(),
		       KernelException,
		       KernelException::KEYVALUEFILE_UNKNOWN_KEY,
		       "KeyValueFile:getItem: unknown key: " << key);

	std::istringstream ss(it->second);
	ss >> value;
      };

      /// set the \a value of item \a key.     
      template<class T>
      void setItem(std::string const& key, T& value) const {
	std::ostringstream ss;
	ss << value;
	keyValue[key] = ss.str();
      };

      /// This method creates the proper KeyValueFile and call the load method of \a loadable
      static void load(KeyValueFileLoad & loadable, std::string const& filename,
		       std::string const& keyValueSeparator_ = ":", char commentPrefix_ = '#');

      /// This method creates the proper KeyValueFile and call the save method of \a loadable
      static void save(KeyValueFileSave & loadable, std::string const& filename,
		       std::string const& keyValueSeparator_ = ":", char commentPrefix_ = '#');

    }; // class KeyValueFile

    /** An object which can be loaded using the KeyValueFile
     * mechanism.
     *
     * \ingroup kernel
     */
    class KeyValueFileLoad {

    protected:

      virtual ~KeyValueFileLoad() {};

      /** Implement this method calling repeatedly
       * KeyValueFile::getItem() method.
       */
      virtual void load(jafar::kernel::KeyValueFile const& keyValueFile) = 0;

      friend class KeyValueFile;
      
    }; // class KeyValueFileLoad


    /** An object which can be saved using the KeyValueFile
     * mechanism.
     *
     * \ingroup kernel
     */
    class KeyValueFileSave {

    protected:

      virtual ~KeyValueFileSave() {};

      virtual void save(jafar::kernel::KeyValueFile const& keyValueFile) = 0;

      friend class KeyValueFile;
      
    }; // class KeyValueFileLoad

    /** An object which can both be loaded and saved using the
     * KeyValueFile mechanism.
     *
     * \ingroup kernel
     */
    class KeyValueFileSaveLoad : public KeyValueFileLoad, public KeyValueFileSave {

      friend class KeyValueFile;
      
    }; // class KeyValueFileLoad

  } // namespace kernel
} // namespace jafar

#endif // KERNEL_KEY_VALUE_FILE_HPP

#ifndef KERNEL_KEY_VALUE_FILE_HPP
#define KERNEL_KEY_VALUE_FILE_HPP

#include <typeinfo>
#include <string>
#include <sstream>
#include <map>

#include "kernel/kernelException.hpp"

namespace jafar {
  namespace kernel {

#ifndef SWIG // swig ignores KeyValueFile class

    /** Read a file containing simple key value statements.
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

      /** Write file \a filename with key:value pairs. The current
       * date is added in comment at the top of the file. 
       *
       * \warning There is no control on the write process, the keys
       * are written in lexical order.
       * 
       */
      void writeFile(std::string const& filename);
      
      bool hasKey(std::string const& key) const;

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

	JFR_IO_STREAM(ss, 
		      "KeyValueFile::getItem: invalid value parsing:" << std::endl << 
		      "key: " << key << std::endl <<
		      "value: " << it->second << std::endl <<
		      "value-type: " << typeid(T).name());
      }

      /// set the \a value of item \a key.     
      template<class T>
      void setItem(std::string const& key, T& value) {
	std::ostringstream ss;
	ss << value;
	keyValue[key] = ss.str();
      }

    }; // class KeyValueFile

#endif // SWIG

    /** Interface of an object which can be loaded using the
     * KeyValueFile mechanism.
     *
     * \ingroup kernel
     */
    class KeyValueFileLoad {

    public:

      /** This method automates the process:
       * - creates an instance of KeyValueFile,
       * - call KeyValueFile::readFile()
       * - call the virtual load method
       */
      void load(std::string const& filename,
		std::string const& keyValueSeparator_ = ":", char commentPrefix_ = '#');

    protected:

      virtual ~KeyValueFileLoad() {};

      /** Implement this method calling repeatedly
       * KeyValueFile::getItem() method.
       */
      virtual void loadKeyValueFile(jafar::kernel::KeyValueFile const& keyValueFile) = 0;
      
    }; // class KeyValueFileLoad


    /** Interface of an object which can be saved using the
     * KeyValueFile mechanism.
     *
     * \ingroup kernel
     */
    class KeyValueFileSave {

    public:

      /** This method automates the process:
       * - creates an instance of KeyValueFile,
       * - call the virtual save method
       * - call KeyValueFile::writeFile()
       */
      void save(std::string const& filename,
		std::string const& keyValueSeparator_ = ":", char commentPrefix_ = '#');

    protected:

      virtual ~KeyValueFileSave() {};

      /** Implement this method calling repeatedly
       * KeyValueFile::setItem() method.
       */
      virtual void saveKeyValueFile(jafar::kernel::KeyValueFile& keyValueFile) = 0;

    }; // class KeyValueFileSave

    /** Interface of an object which can both be loaded and saved
     * using the KeyValueFile mechanism.
     *
     * \ingroup kernel
     */
    class KeyValueFileSaveLoad : public KeyValueFileLoad, public KeyValueFileSave {

    }; // class KeyValueFileSaveLoad

  } // namespace kernel
} // namespace jafar

#endif // KERNEL_KEY_VALUE_FILE_HPP

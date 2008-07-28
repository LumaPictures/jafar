#ifndef CSV_FILE_HPP
#define CSV_FILE_HPP

#include <typeinfo>
#include <string>
#include <sstream>
#include <map>
#include "boost/numeric/ublas/matrix.hpp"
#include "kernel/kernelException.hpp"

namespace jafar {
  namespace kernel {

    /** Read a file containing comma separated values statements with or without first line as title.
     *
     * \ingroup kernel
     */
    class CSVFile {

      private:

      /// columns separator, default to " ".
      std::string separator;
      /// prefixes comment line
      char commentPrefix;
      /// indicates if columns names are given
      bool withColumnsNames;
      /// map column name to an integer
      std::map<std::string, int> columnNames;
      typedef boost::numeric::ublas::matrix<std::string> stringMatrix;
      /// matrix used to save values
      stringMatrix fileMatrix;

      public:

      CSVFile(std::string const& separator_ = " ", char commentPrefix_ = '#');

      /// Parse file \a filename for comma separated values lines.
      void readFile(std::string const& filename);

      /** Write file \a filename with comma separated values. The current
       * date is added in comment at the top of the file. 
       */
      void writeFile(std::string const& filename);
      /// specify whether the first line is for column names or not.
      /// \Warning It must use the same separator as the data.
      void hasColumnsNames(const bool& _withColumnsNames);
      /// indicates whether this \a columnName exists 
      bool hasColumn(std::string const& columnName);
      /// returns the number of lines loaded
      uint nbOfLines();
      /// returns the number of columns per line
      uint nbOfColumns();
      /// get the \a value at \a columnNumber at \a line.
      template<class T>
        void getItem(const int& line, const int& columnNumber, T& value) const {

        JFR_PRED_ERROR(line >= 0 && line <= fileMatrix.size1(),
                       KernelException,
                       KernelException::CSVFILE_UNKNOWN_LINE,
                       "CSVFile:getItem: unknown line: " << line);
        
        JFR_PRED_ERROR(columnNumber >= 0 && columnNumber <= fileMatrix.size2(),
                       KernelException,
                       KernelException::CSVFILE_UNKNOWN_COLUMN,
                       "CSVFile:getItem: unknown column: " << columnNumber);

        std::istringstream ss(fileMatrix(line, columnNumber));
        ss >> value;

        JFR_IO_STREAM(ss, 
                      "CSVFile::getItem: invalid value parsing:" << std::endl << 
                      "line: " << line << std::endl <<
                      "column: " << columnNumber << std::endl <<
                      "value: " <<  fileMatrix(line, columnNumber) << std::endl <<
                      "value-type: " << typeid(T).name());
      }	

      /// get the \a value of item \a columnName at \a line.
      template<class T>
        void getItem(const int& line, const std::string& columnName, T& value) const {
        std::map<std::string,int>::const_iterator iter = columnNames.find(columnName);
	JFR_PRED_ERROR(iter != columnNames.end(),
		       KernelException,
		       KernelException::CSVFILE_UNKNOWN_COLUMN_NAME,
		       "CSVFile:getItem: unknown column name: " << columnName);

        int colNbr = iter->second;
        getItem<T>(line, colNbr, value);
      }

      /// set the \a value at \a column and \a line.     
      template<class T>
        void setItem(int line, const int& column, T& value) {
	std::ostringstream ss;
	ss << value;
        fileMatrix(line, column) = ss.str();
      }

      /// set the \a value of item \a column at \a line.     
      template<class T>
        void setItem(const int& line, const std::string& colName, T& value) const {
        std::map<std::string,int>::const_iterator iter = columnNames.find(colName);
	JFR_PRED_ERROR(iter != columnNames.end(),
		       KernelException,
		       KernelException::CSVFILE_UNKNOWN_COLUMN_NAME,
		       "CSVFile:getItem: unknown column name: " << colName);

        int colNbr = iter->second;
        setItem<T>(line, colNbr, value);
      }

    }; // class CSVFile

    /** Interface of an object which can be loaded using the
     * CSVFile mechanism.
     *
     * \ingroup kernel
     */
    class CSVFileLoad {

      public:

      /** This method automates the process:
       * - creates an instance of CSVFile,
       * - call CSVFile::readFile()
       * - call the virtual load method
       */
      void load(std::string const& filename,
		std::string const& separator_ = " ", char commentPrefix_ = '#');

      protected:

      virtual ~CSVFileLoad() {};

      /** Implement this method calling repeatedly
       * CSVFile::getItem() method.
       */
      virtual void loadCSVFile(jafar::kernel::CSVFile& csvFile) = 0;
      
    }; // class CSVFileLoad


    /** Interface of an object which can be saved using the
     * CSVFile mechanism.
     *
     * \ingroup kernel
     */
    class CSVFileSave {

      public:

      /** This method automates the process:
       * - creates an instance of CSVFile,
       * - call the virtual save method
       * - call CSVFile::writeFile()
       */
      void save(std::string const& filename,
		std::string const& separator_ = " ", char commentPrefix_ = '#');

      protected:

      virtual ~CSVFileSave() {};

      /** Implement this method calling repeatedly
       * CSVFile::setItem() method.
       */
      virtual void saveCSVFile(jafar::kernel::CSVFile& csvFile) = 0;

    }; // class CSVFileSave

    /** Interface of an object which can both be loaded and saved
     * using the CSVFile mechanism.
     *
     * \ingroup kernel
     */
    class CSVFileSaveLoad : public CSVFileLoad, public CSVFileSave {

    }; // class CSVFileSaveLoad

  } // namespace kernel
} // namespace jafar

#endif // KERNEL_KEY_VALUE_FILE_HPP

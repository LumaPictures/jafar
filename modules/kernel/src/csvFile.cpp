/* $Id$ */

#include <fstream>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/tokenizer.hpp"

#include "kernel/csvFile.hpp"

using namespace jafar::kernel;

/*
 * class CSVFileReader
 */

CSVFile::CSVFile(std::string const& separator_, char commentPrefix_) :
  separator(separator_),
  commentPrefix(commentPrefix_)
{}

void CSVFile::hasColumnsNames(const bool&  _withColumnsNames) {
  withColumnsNames = _withColumnsNames;
}

bool CSVFile::hasColumn(const std::string& _columnName) {
  std::map<std::string, int>::const_iterator it = columnNames.find(_columnName);
  return it != columnNames.end();
}

uint CSVFile::nbOfLines() {
  return fileMatrix.size1();
}

uint CSVFile::nbOfColumns() {
  return fileMatrix.size2();
}

void CSVFile::readFile(std::string const& filename) {
  using namespace std;

  ifstream file(filename.c_str());

  JFR_IO_STREAM(file, "CSVFile:readFile: failed to open file: " << filename);

  int lineNumber = 0;
  int nonCommentLineNumber = 0;
  int columnNumber = 0;

  // reading the lines
  string line;

  // tokenizer
  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep(separator.c_str());

  while(getline(file,line)) {
    lineNumber++;

    // this is an empty line
    // or a comment
    if (line.size() == 0 || line.at(0) == commentPrefix || ( line.size() == 1 && line.at(0) == 13 ) )
      continue;
	  
    tokenizer tokens(line, sep);
    tokenizer::iterator it = tokens.begin();
    ++nonCommentLineNumber;
    JFR_PRED_ERROR(it != tokens.end(),
		   KernelException,
		   KernelException::CSVFILE_INVALID_LINE,
		   "CSVFile:readFile: invalid line at " << filename << ":" << lineNumber);
    //treat the title line
    if(withColumnsNames && nonCommentLineNumber == 1){
      for(it = tokens.begin(), columnNumber = 0; 
          it != tokens.end(); 
          ++it, ++columnNumber) {
        columnNames[*it] = columnNumber;
      }
      //generate column names if no title line and treat data
    } else if (!withColumnsNames && nonCommentLineNumber == 1) {
      std::stringstream ss;
      for(it = tokens.begin(), columnNumber = 0; 
          it != tokens.end(); 
          ++it, ++columnNumber) {
        ss << columnNumber;
        columnNames[ss.str()] = columnNumber;
        ss << std::flush;
        fileMatrix(nonCommentLineNumber, columnNumber) = *it;
      }
      //treat data line
    } else {
      for(it = tokens.begin(), columnNumber = 0; 
          it != tokens.end(); 
          ++columnNumber, ++it) {
        fileMatrix(nonCommentLineNumber, columnNumber) = *it;
      }
    }
  }//while loop
}

void CSVFile::writeFile(std::string const& filename) {

  std::ofstream file(filename.c_str());
  JFR_IO_STREAM(file, "CSVFile:writeFile: failed to open file: " << filename);

  file << commentPrefix <<" Generated by CSVFile::writeFile" << std::endl;
  file << commentPrefix <<" " << boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time()) << std::endl;
  file << std::endl;

  for (uint line = 0; line < fileMatrix.size1(); ++line) {
    for (uint col = 0; col < fileMatrix.size2()-1 ; ++col) {
      file << fileMatrix(line, col) << separator;
    }
    //writes the last column of line and inserts a break
    file <<  fileMatrix(line , fileMatrix.size2()-1)<< std::endl;
  }
}

/*
 * class CSVFileLoad
 */
void CSVFileLoad::load(std::string const& filename,
                       std::string const& separator_, 
                       char commentPrefix_) {
  JFR_TRACE_BEGIN;
  CSVFile csvf(separator_, commentPrefix_);
  csvf.readFile(filename);
  loadCSVFile(csvf, csvf.nbOfLines(), csvf.nbOfColumns());
  JFR_TRACE_END("CSVFile:load: loaded file " << filename);
}

/*
 * class CSVFileSave
 */
void CSVFileSave::save(std::string const& filename,
                       std::string const& separator_, 
                       char commentPrefix_) {
  JFR_TRACE_BEGIN;
  CSVFile csvf(separator_, commentPrefix_);
  saveCSVFile(csvf);
  csvf.writeFile(filename);
  JFR_TRACE_END("CSVFile:load: saved file " << filename);
}


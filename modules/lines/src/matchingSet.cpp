#include "lines/matchingSet.hpp"
using namespace std;
using namespace jafar;
using namespace lines;

MatchingSet::MatchingSet(MatchRepres _matchRep){
  matchRep = _matchRep;
  maxInd = 0;
 
}

void MatchingSet::addMatching(int indA, int indB){
  if(matchRep==FAST_OLD_LINE_MATCH_ACCESS || matchRep == BOTH_MATCH_ACCESS_TYPES){
    // if index A is greater than max, then there are some indexes smaller than indA without a matching yet
    // therefore some dummy vectors are insert
    vector<int> noMatch;
    noMatch.push_back(-1); 
    
    if(indA >= maxInd) map.resize(indA+1);
    
    while(indA >= maxInd){
      map[maxInd] = noMatch;
      maxInd++;
    }
    
    // map[indA][0] exits now, but maybe there are other matchings yet
    if(map[indA][0] == -1){
      map[indA][0] = indB;
    }
    else{
      map[indA].push_back(indB);
    }
  }
  
  if(matchRep==FAST_MATCH_ACCESS || matchRep == BOTH_MATCH_ACCESS_TYPES){
    oldIdx.push_back(indA);
    newIdx.push_back(indB);
  }
  
}
  

/*
bool MatchingSet::load(QString filename){
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
    cout << "pERROR: line matching file can not be loaded\n";
    return 0;
  }

  // clear old matching
  clear();
  
  QTextStream in(&file); 
  
  maxInd=0;
  
  // read first line
  QString line = in.readLine();
  while(!line.isNull()){
    QStringList list = line.split(" ", QString::SkipEmptyParts);
    vector<int> newSet;

    // ignore comments
    if(list.at(0) != "#"){
      
      // fill the map at index with the possible matchings
      int i=1;
      while(i<list.size()){
        newSet.push_back(list.at(i).toInt());
        i++;
      }
      
      // add the matching vector to map
      map.push_back(newSet);
      maxInd++;
    }
    // read next line
    line = in.readLine();
  }
  

  // uncomment to output the matching on stdout
  //cout << "map.size=" << map.size() << "  map[0].size=" << map[0].size() << endl;
  //for(int i=0; i< map.size(); i++){
  //  cout << i << endl;
  //  for(int j=0; j<map[i].size(); j++){
  //    cout << "\t" << map[i][j] << endl;
  //  }
  //}
  
  file.close();

  return 1;
}
*/

/*
bool MatchingSet::save(QString filename){
  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)){
    cout << "pERROR: line matching file can not be saved\n";
    return 0;
  }

  QTextStream out(&file);
  out << "# matching" << endl;
  out << "# -1 indicates there is no right matching, all old indexes must appear!" << endl;
  out << "# format: oldIndex newIndex1 newIndex2 newIndex3 ..." << endl;
    
  for(uint i=0; i<map.size(); i++){
    out << i;
    for(uint j=0; j<map[i].size(); j++){
      out << " " << map[i][j];
    }
    out << endl;
  }
  file.close();
  return 1;
}
*/



bool MatchingSet::searchMatching(int indA, int indB){
  if(matchRep==FAST_OLD_LINE_MATCH_ACCESS || matchRep == BOTH_MATCH_ACCESS_TYPES){
      
    for(uint i=0; i<map[indA].size(); i++){
      if(map[indA][i] == indB) return 1;
    }
    return 0;
  }
  else{
    for(uint i=0; i<oldIdx.size(); i++){
      if(oldIdx[i] == indA && newIdx[i] == indB) return 1;
    }
    return 0;
  }
}

void MatchingSet::print(){
  if(matchRep == FAST_MATCH_ACCESS){
    cout << "Matching has to stored with FAST_OLD_LINE_MATCH_ACCESS\n";
    return;
  }
  
  cout << "matching size: " << map.size()<< endl;
  for(uint i=0; i< map.size(); i++){
    cout << "[" << i <<"]:\t";
    for(uint j=0; j<map[i].size(); j++){
      cout << "\t" << map[i][j];
    }
    cout << endl;
  }
}

void MatchingSet::clear(){
  for( uint i=0; i<map.size(); i++) 
    map[i].clear();
  map.clear();
  
  oldIdx.clear();
  newIdx.clear();
  
}


void MatchingSet::compareMatchingSets( MatchingSet* newMatching, MatchingSet* refMatching, MatchStatistic* statistic){
  
  statistic->right=0;
  statistic->wrong=0;
  statistic->notFound=0;
  
  // flag to indicate if current matching is equal
  int found = 0;

  /////// calculate "right" ///////////
  // for all matchings in newMatching
  for(int i=0; i<newMatching->getMaxInd(); i++){
    found=0;
    // search every possible matching in refMatching
    for(uint j=0; j<newMatching->map[i].size(); j++){
      if(refMatching->searchMatching(i, newMatching->map[i][j])) found=1;
    }
    // if at least one is found score as right matching
    if(found) statistic->right++;
  }
  
  /////// calculate "wrong" ///////////
  // for all matchings in newMatching
  for(int i=0; i<newMatching->getMaxInd(); i++){
    found=0;
    // search every possible matching in refMatching
    for(uint j=0; j<newMatching->map[i].size(); j++){
      if(refMatching->searchMatching(i, newMatching->map[i][j])) found=1;
    }
    // if nothing is found score as wrong matching
    if(!found) statistic->wrong++;
  }
  
  /////// calculate "notFound" ///////////  
  // for all matchings in newMatching
  for(int i=0; i<newMatching->getMaxInd(); i++){
    // if there is no matching in newMatching but at least one in refMatching count as "not found" 
    if( (newMatching->map[i][0] == -1) && (refMatching->map[i][0] != -1) ){
      statistic->notFound++;
    }
  }
  
  //cout << "number right: " << statistic->right << endl;
  //cout << "number wrong: " << statistic->wrong << endl;
  //cout << "number notFound: " << statistic->notFound << endl;


}

 void MatchingSet::clone(MatchingSet& old, MatchingSet& clone){
    clone.clear();
    
    vector<int> dummy;
    
    for(uint i=0; i<old.map.size(); i++){
      dummy.clear();
      for(uint j=0; j<old.map[i].size(); j++){
        dummy.push_back(old.map[i][j]);
      }
      clone.map.push_back(dummy);
    }
    
  }
  
  


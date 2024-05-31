#include "cpp_package_barebones/type_definitions.h"
#include "cpp_package_barebones/SalesItem.h"

#include <utility>      // std::pair, std::make_pair
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

#include <memory>  // std::shared_ptr


std::pair< std::string, std::vector<double> >
read_input(const std::string& file_name)
{
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    if( config["isbn"] && config["units_revenue"] )
      return std::make_pair( config["isbn"].as<std::string>(), config["units_revenue"].as<std::vector<double>>() );
  } catch (YAML::ParserException& e) {
    std::cout << e.what() << "\n";
  }
  
  printf(ANSI_COLOR_RED "Check input format!\n" ANSI_COLOR_RESET);
  return std::pair< std::string, std::vector<double> >();
}


int main(int argc, char ** argv)
{

  if(argc != 2)
  {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  
  // Read SalesItem data
  std::pair< std::string, std::vector<double> > A(read_input(argv[1]));
  std::cout << "Read:" << std::endl << A.first << " " << A.second[0] << " " << A.second[1] << " " << std::endl;

  // Construct a SalesItem object
  erl::SalesItem S1(A.first, A.second[0], A.second[1]);
  
  // Assign S1 to S2
  erl::SalesItem S2 = S1; 

  // Display S2  
  std::cout << "SalesItem Object S2: " << S2 << std::endl;

  // Make a pointer to S1
  std::shared_ptr<erl::SalesItem> pS1 = std::make_shared<erl::SalesItem>(S1);
  
  // Display S1
  std::cout << "SalesItem Object S1: " << *pS1 << std::endl;
  std::cout << "S1.isbn = " << pS1->get_isbn() << std::endl;
  
  return 0;
}

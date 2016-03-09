// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <upperPlanner_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class upperPlanner_IDL_re_plan : public yarp::os::Portable {
public:
  double _deadline;
  bool _return;
  void init(const double _deadline);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool upperPlanner_IDL_re_plan::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("re_plan",1,2)) return false;
  if (!writer.writeDouble(_deadline)) return false;
  return true;
}

bool upperPlanner_IDL_re_plan::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void upperPlanner_IDL_re_plan::init(const double _deadline) {
  _return = false;
  this->_deadline = _deadline;
}

upperPlanner_IDL::upperPlanner_IDL() {
  yarp().setOwner(*this);
}
bool upperPlanner_IDL::re_plan(const double _deadline) {
  bool _return = false;
  upperPlanner_IDL_re_plan helper;
  helper.init(_deadline);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool upperPlanner_IDL::re_plan(const double _deadline)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool upperPlanner_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "re_plan") {
      double _deadline;
      if (!reader.readDouble(_deadline)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = re_plan(_deadline);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> upperPlanner_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("re_plan");
    helpString.push_back("help");
  }
  else {
    if (functionName=="re_plan") {
      helpString.push_back("bool re_plan(const double _deadline) ");
      helpString.push_back("Require replan with new time constraint ");
      helpString.push_back("@param _deadline Time constraint ");
      helpString.push_back("           (put it between brackets if asking for it through rpc). ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}



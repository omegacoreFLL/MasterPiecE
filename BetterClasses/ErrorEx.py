class ErrorEx():

    @staticmethod
    def isType(obj, name, required_types):
        throw = False

        if not isinstance(name, str):
            ErrorEx.__throw("name", [str])
        elif not isinstance(required_types, list):
            if not isinstance(obj, required_types):
                throw = True
        else:
            throw = True
            for Type in required_types:
                if isinstance(obj, Type):
                    throw = False
            
        if throw:
            ErrorEx.__throw(name, required_types)
    
    @staticmethod
    def __throw(name, required_types):
        raise Exception("Not a valid ---{0}---. Needed type(s): {1}".format(name, required_types))
    
    
            
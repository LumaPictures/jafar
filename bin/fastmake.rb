#!/usr/bin/ruby

class Builder
  def initialize()
    f = File.new("#{ENV['JAFAR_DIR']}/Module.config")
    moduleConfig = f.read
    scriptLanguages = moduleConfig[/SCRIPT_LANGUAGES = .*/]
    @scriptLanguages = scriptLanguages.split(/=/)[1]
    @buildedModule = []
  end

  def buildModule(name)
    if(@buildedModule.include?(name))
      return true
    end
#     puts "==== Looking for dependency for #{name} ===="
    f = File.new("#{moduleDir(name)}/User.make")
    userMake = f.read
    requiredModules = userMake[/REQUIRED_MODULES =.*/]
    requiredModules = requiredModules.split(/=/)[1].split(/ /)
    requiredModules.each() { |m|
      if(m != "")
        if(not buildModule(m))
          return false
        end
      end
    }
    optionalModules = userMake[/OPTIONAL_MODULES =.*/]
    if(optionalModules.nil?)
      puts "WARNING: no OPTIONAL_MODULES found in #{moduleDir(name)}/User.make !"
    else
      optionalModules = optionalModules.split(/=/)[1]
      if(not optionalModules.nil?)
        optionalModules.split(/ /).each() { |m|
          if(m != "" and File.exist?(moduleDir(m)))
            if(not buildModule(m))
              return false
            end
          else
            puts "Optional module #{File.exist?(moduleDir(m))} not found."
          end
        }
      end
    end
#     puts "==== Building #{name} ===="
    return callMake(name)
  end
private
  def callMake(name)
    if(not File.exist?(moduleDir(name)))
      return false
    end
    if(not @buildedModule.include?(name))
      @buildedModule << name
      return system("cd #{moduleDir(name)};make dirs lib-install #{@scriptLanguages}")
    end
    return true
  end
  def moduleDir(name)
    return "#{ENV['JAFAR_DIR']}/modules/#{name}/"
  end
end


b = Builder.new
exit(b.buildModule(File.basename(Dir.pwd)))

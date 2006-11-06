#!/usr/bin/ruby

if(ARGV.size != 1)
  puts "jafar-co-module.rb [modulename]"
  puts "This scripts will attempt to checkout"
  puts "a specific module and its dependencies"
  exit
end

@optionalpackages = ""
@allreadychecked = ""

def checkoutDependency(name)
  if(@allreadychecked.index(name) != nil)
    return true
  end
  @allreadychecked << " " << name
  puts "Checking dependencies of #{name}"
  usermake = "#{name}/User.make"
  if(File.exist?(usermake))
    open(usermake) do |file|
      file.each_line do |line|
        if(line.index("REQUIRED_MODULES") == 0 )
          line.chop!
          line.sub!("REQUIRED_MODULES", "")
          line.split(/ |=/).each do |x|
            if(x != "" and !checkoutModule( x ))
                return false
            end
          end
        elsif(line.index("OPTIONAL_MODULES") == 0)
          line.chop!
          line.sub!("OPTIONAL_MODULES", "")
          line.split(/ |=/).each do |x|
            if(x != "" and @optionalpackages.index(x) == nil)
              @optionalpackages << " " << x
            end
          end
        end
      end
    end
    return true
  else
    puts "No file #{usermake}, dependency for #{name} can't be checked."
    return true
  end
end

def checkoutModule(name)
#first check if the module has allready been checkout
  if(File.exist?(name))
    return checkoutDependency(name)
  else
    puts "Checkout module : |#{name}|"
    url="svn+ssh://#{ENV['USER']}@svn.laas.fr/svn/jafar/jafarModules/trunk/#{name}"
    `svn co #{url} 2>/dev/null`
    
    if( File.exist?(name) )
      return checkoutDependency(name)
    else
      puts "An error has occured when checkouting module : #{name}, please check that it exists"
      return false
    end
  end
end

if(checkoutModule(ARGV[0]))
  puts "Module #{ARGV[0]} and all its dependencies have been successfully"
  puts "installed, you can now run make in the directory module/#{ARGV[0]}"
  if(@optionalpackages != "")
    puts ""
    puts "Optional dependencies that you may want to install : #{@optionalpackages}"
  end
else
  puts "An error has occured."
end

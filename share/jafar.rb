module Jafar
    TOPDIR = ENV['JAFAR_DIR'] || File.expand_path('..', File.dirname(__FILE__))
    class << self
        ## Autodetects the system type as used by Jafar
        def detect_system_type
            config_dir = File.join(TOPDIR, 'config')
            guess = File.join(config_dir, 'config.guess')
            sub = File.join(config_dir, 'config.sub')

            unless sysinfo = IO.popen(guess) { |out| out.read }.chop
                raise "Cannot guess the system type"
            end

            sysinfo = IO.popen("#{sub} #{sysinfo}") { |out| out.read }.chop
            sysinfo = sysinfo.split('-')
            [sysinfo[0], sysinfo[2], sysinfo[3]].join('-')
        end

        ## Sets load path using the system type
        attr_reader :system_type
        attr_reader :library_path
        def system_type=(system_type)
            @system_type = system_type
            @library_path = File.join(TOPDIR, "lib/#{system_type}/ruby/#{Config::CONFIG['ruby_version']}")
            $LOAD_PATH << @library_path
        end
    end
end

Jafar.system_type = Jafar.detect_system_type

puts ""
puts "Welcome to jafar !         (c) 2004-2005 LAAS-CNRS"
puts "jafar_dir: #{Jafar::TOPDIR}"
puts ""

puts "*** Loading kernel module"
require 'jafar/kernel'
print "*** Available modules: "
puts Dir[File.join(Jafar.library_path, "jafar/*.rb")].map { |p| File.basename(p, '.rb') }.join(', ')

puts ""


require 'jafar/kernel/kernel'

module Jafar
    class << self
        attr_writer :auto_include
        def auto_include?; @auto_include end
    end

    ## Register a new SWIG-generated Jafar module
    # - it defines the Rb* exception classes from the Jafar Exception classes
    # 
    # If auto_include is set, then the module is automatically included
    # in the top-level. This is to be used in interactive shells for instance.
    def self.register_module(jafar_module)
        # Register the exception classes defined in jafar_module
        module_prefix = Regexp.new("^#{jafar_module.name}::")
        ObjectSpace.each_object do |klass|
            next unless Class === klass
            next unless klass.ancestors.find { |parent| parent == Jafar::Kernel::Exception }
            next unless match_data = module_prefix.match(klass.name)

            cxx_name = match_data.post_match
            unless jafar_module.const_defined?('Rb' + cxx_name)
                register_exception(jafar_module, cxx_name)
            end
        end

        include jafar_module if auto_include?
    end

    ## Alias the module methods using these rules:
    #   getCamelCase => camel_case
    #   setCamelCase => camel_case=
    #   isCamelCase  => camel_case?
    #   blaCamelCase    => bla_camel_case
    def self.rename_methods(method)
        method
    end
    
    ## Creates a Ruby exception from the Jafar one
    #
    def self.register_exception(jafar_module, cxx_name, rb_base = Jafar::Kernel::RbException)
        cxx = jafar_module.const_get(cxx_name)
        # Check that it is not already defined
        return unless 

        # Create the Ruby-side exception class
        rb_name = 'Rb' + cxx_name
        rb = Class.new(rb_base) do
            attr_reader :id
            def initialize(id)
                @id = self.class.id_to_symbol(id)
                super
            end

            ## Hack! The C function rb_raise does not
            # allow to do a rb_raise(object), while
            # the interpreter does. This fixes it
            alias :new :exception

        private
            @@exception_id_map = {}
            cxx.constants.each do |name|
                value = cxx.const_get(name)
                if Numeric === value
                    @@exception_id_map[value] = name.intern
                end
            end
            def self.id_to_symbol(id)
                if symbol = @@exception_id_map[id]
                    symbol
                else
                    :UNKNOWN
                end
            end
        end
        jafar_module.const_set(rb_name, rb)
        cxx.const_set(:RubyException, rb_name)
    end
end

Jafar.register_exception Jafar::Kernel, 'Exception', Exception
class Jafar::Kernel::Exception
    def to_s
        self.what
    end
end
Jafar.register_module Jafar::Kernel


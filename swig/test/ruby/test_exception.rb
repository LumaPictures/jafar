require 'test/unit'

require 'jafar/helloworld'
include Jafar::Helloworld

class TC_Exceptions < Test::Unit::TestCase
    def test_cxx_to_ruby
        # Test the simple exception forwarding
        assert_raise(RbHelloworldException) { HelloWorld.new.setHello "bla" }

        begin
            HelloWorld.new.setHello "bla"
        rescue RbHelloworldException => e
            # Make sure the exception id gets forwarded
            assert_equal(:BAD_FORMAT, e.id)
        end
    end
end


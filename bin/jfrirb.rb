#!/usr/bin/ruby
require 'Qt'

Qt::Application.new(ARGV)
Qt::Application.setQuitOnLastWindowClosed(false)

class Handler < Qt::Object
  slots "pass_thread(void)"
  def pass_thread
    Thread.pass
  end
end

handler = Handler.new
timer = Qt::Timer.new
timer.connect(timer, SIGNAL("timeout()"), handler, SLOT("pass_thread()"))

Thread.abort_on_exception = true
Thread.new do
    timer.start(0)
    $qApp.exec
    STDERR.puts "WARNING: The graphical application has been closed ! You need to restart the jafar shell !"
end


# launch irb
require 'irb'
require 'irb/completion.rb'

IRB.setup(nil)

include Jafar

ws  = IRB::WorkSpace.new(binding)
irb = IRB::Irb.new(ws)
IRB.conf[:MAIN_CONTEXT] = irb.context

trap("SIGINT") do
    irb.signal_handle
end

catch(:IRB_EXIT) do
    irb.eval_input
end

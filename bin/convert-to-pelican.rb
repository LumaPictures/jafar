#!/usr/bin/ruby

require "jafar/geom"

include Jafar

def printHelp
  puts "convert-to-pelican.rb [options] src dst"
  puts "This scripts convert old data storage to follow the pelican specification"
  puts " --compress type : where type is {NONE, ZIP}, the default is NONE"
  puts " --format type : the format of the source, the default is oldpelican"
  puts "    oldpelican"
  puts "    calife"
  puts ""
  puts " --help : print this help message and exit"
  exit
end

def genNumber(n, mins)
  s = n.to_s
  while(s.size < mins)
    s = "0#{s}"
  end
  return s
end

def createT3DfromDala(s)
  scaned = s.scan(/\d+.\d+/)
  t3d = Geom::T3DEuler.new
  t3d.setValue(scaned[0].to_f, scaned[1].to_f, scaned[2].to_f, scaned[3].to_f, scaned[4].to_f, scaned[5].to_f)
  return t3d
end

def fromOldPelican(dirsrc, dirdst, compress)
    puts "Initialize the config directory"
    if(File.exist?("#{dirsrc}/config"))
      File.copy("#{dirsrc}/config", dirdst)
    else
      puts "WARNING: no config directory found, you will need to manually add it to the directory"
    end
    puts "Estimate the first and last image number"
    startn = -1
    endn = -1
    n = 0
    while(true)
      filename = "#{dirsrc}/images/image.#{genNumber(n,3)}.g"
      if(not File.exist?(filename) and startn != -1 )
        endn = n - 1
        break
      end
      if(File.exist?(filename) and startn == -1 )
        startn = n
      end
      if(startn == -1 and n > 1000)
        puts "no image files found"
        exit
      end
      n += 1
    end
    puts "First number: #{startn} Last number: #{endn}"
    puts "Copying and converting pos files"
    if(File.exist?("#{dirsrc}/images/image.pos.#{genNumber(startn,3)}.g"))
      basepos="#{dirdst}/pos/"
      Dir.mkdir(basepos)
      for i in startn..endn
        print "\r#{i-startn}/#{endn-startn}"
        $stdout.flush
        numb = genNumber(i,3)
        filename = "#{dirsrc}/images/image.pos.#{numb}.g"
        f = File.new(filename, "r")
        lines = f.readlines
        t3d = createT3DfromDala(lines[2])
        t3d.save("#{basepos}sensorToMain.#{numb}.l.t3d")
        t3d = createT3DfromDala(lines[3])
        t3d.save("#{basepos}mainToBase.#{numb}.t3d")
        t3d = createT3DfromDala(lines[4])
        t3d.save("#{basepos}mainToOrigin.#{numb}.t3d")
        filename = "#{dirsrc}/images/image.pos.#{numb}.d"
        f = File.new(filename, "r")
        lines = f.readlines
        t3d = createT3DfromDala(lines[3])
        t3d.save("#{basepos}sensorToMain.#{numb}.r.t3d")
      end
      print "\n"
    else
      puts "No pos file available, skiping"
    end
    puts "Copying and converting images"
    Dir.mkdir("#{dirdst}/images")
    for i in startn..endn
      numb = genNumber(i,3)
      filename = "#{dirsrc}/images/image.#{numb}.g"
      print "\r#{i-startn}/#{endn-startn}"
      $stdout.flush
      `convert #{dirsrc}/images/image.#{numb}.g -compress #{compress} #{dirdst}/images/image.#{numb}.l.tiff`
      `convert #{dirsrc}/images/image.#{numb}.d -compress #{compress} #{dirdst}/images/image.#{numb}.r.tiff`
    end
    print "\n"
    puts "Write data configuration"
    f = File.new("#{dirdst}/data.cfg","w")
    f.puts("nbSensors: 1")
    f.puts("sensor0Name: stereoFront")
    f.puts("sensor0Type: stereo")
    f.puts("sensor0NbDigits: 3")
    f.puts("sensor0CamLeft: l")
    f.puts("sensor0CamRight: r")
    f = File.new("#{dirdst}/images/l.cfg","w")
    f.puts("nbDigits: 3")
    f.puts("first: #{startn}")
    f.puts("last: #{endn}")
    f.puts("step: 1")
    f = File.new("#{dirdst}/images/r.cfg","w")
    f.puts("nbDigits: 3")
    f.puts("first: #{startn}")
    f.puts("last: #{endn}")
    f.puts("step: 1")
end

compress = "NONE"
format = "oldpelican"

pos = 0

while(true)
  if(ARGV[pos][0,2] != "--")
    break
  end
  case ARGV[pos]
    when "--compress"
      pos += 1
      compress = ARGV[pos]
      if(compress != "NONE" and compress != "ZIP")
        puts "Unsuported compression"
        puts ""
        printHelp()
      end
    when "--format"
      pos += 1
      format = ARGV[pos]
      if(format != "oldpelican" and format != "calife")
        puts "Unsuported format"
        puts ""
        printHelp()
      end
      if(format == "calife")
        puts "Unimplemented"
        exit
      end
  end
  pos += 1
end

if(ARGV.size - pos < 2)
  puts "Not enought arguments ! You need to specify the source and destination directory"
  puts " "
  printHelp()
end
require 'ftools'
puts "Compression is set to #{compress} and format to #{format}"


dirsrc = ARGV[pos]
pos+=1
dirdst = ARGV[pos]

puts "convert from #{dirsrc} to #{dirdst}"

if(not File.exist?(dirsrc))
  puts "#{dirsrc} doesn't exist"
  exit
end

if(File.ftype(dirsrc) != "directory")
  puts "#{dirsrc} must be a directory"
  exit
end

if(File.exist?(dirdst))
  puts "#{dirdst} exist"
  exit
end

puts "Create #{dirdst}"
Dir.mkdir(dirdst)

case format
  when "oldpelican"
    fromOldPelican(dirsrc, dirdst, compress)
end

puts "Convertion is finished"

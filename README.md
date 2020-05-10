This is a pure python port of helios_dac (https://github.com/Grix/helios_dac) interface code.

i just wanted something a bit lighter-weight to drive the vector display, probably not what 
you want to drive your laser show

loads ILDA files and palettes, for doing the animations and stuff


<pre>
ok.. so this does do some stuff now, and might be helpful for you.. the interface may change as i actually work with this so YMMV.

heres a quick howto:

displaying text using the hershy font generator:

	cal = a.generateText("hello World", 0, 0,scale=10)
	pps = 20000
	while(1):
		a.newFrame(pps,cal)
		a.DoFrame()

displaying an ILDA file:
	cal = a.loadILDfile("ildatest.ild")
	pps = 20000
	while(1):
		for (t,n1,n2,c),f in cal:
			print("playing %s,%s, %d" % (n1,n2,c))
			a.newFrame(pps,f)
			a.DoFrame()

manual drawing:
	pps = 20000
	while(1):
		a.newFrame(pps,[HeliosPoint(0,200), #draw a square
				HeliosPoint(200,200),
				HeliosPoint(200,0),
				HeliosPoint(0,0)])
		a.DoFrame()


these all assume you want direct control over when the frames are sent,but there is a thread you can invoke to handle the displaying of frames
for you and may make more dynamic setups easier:

	a = HeliosDAC()
	a.runQueueThread()
	cal = a.loadILDfile("astroid.ild")
	pps = 20000
	while(1):
		for (t,n1,n2,c),f in cal:
			print("playing %s,%s, %d" % (n1,n2,c))
			a.newFrame(pps,f)


heliospy phar$ python asttest.py 
playing b'Astroid.',b'MediaLas', 0
playing b'Astroid.',b'MediaLas', 1
playing b'Astroid.',b'MediaLas', 2
playing b'Astroid.',b'MediaLas', 3
playing b'Astroid.',b'MediaLas', 4
playing b'Astroid.',b'MediaLas', 5
playing b'Astroid.',b'MediaLas', 6
playing b'Astroid.',b'MediaLas', 7
playing b'Astroid.',b'MediaLas', 8
playing b'Astroid.',b'MediaLas', 9
playing b'Astroid.',b'MediaLas', 10
playing b'Astroid.',b'MediaLas', 11
playing b'Astroid.',b'MediaLas', 12

</pre>
todo:
	drop points that are off the screen due to math

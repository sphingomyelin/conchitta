CFLAGS = -fPIC -g
LIBS =  `pkg-config opencv --cflags --libs`


% : %.cpp
	g++ $(CFLAGS) $< -o $@ $(LIBS)

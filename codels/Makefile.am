lib_LTLIBRARIES = librmp400_codels.la

librmp400_codels_la_SOURCES=	rmp400_c_types.h
librmp400_codels_la_SOURCES+=	rmp400_codels.cc
librmp400_codels_la_SOURCES+=	rmp400_MotionTask_codels.cc
librmp400_codels_la_SOURCES+=	rmp400_TrackTask_codels.cc

librmp400_codels_la_SOURCES+=	codels.h joystick.cc motion_helpers.cc \
				odoProba.cc odoProba.h orMathLib.h

librmp400_codels_la_SOURCES+=	rmp400_Log.h rmp400_Log.cc

librmp400_codels_la_CPPFLAGS=	-Wall $(requires_CFLAGS)
librmp400_codels_la_LDFLAGS=	$(requires_LIBS)
librmp400_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	rmp400_c_types.h
CLEANFILES=	${BUILT_SOURCES}
rmp400_c_types.h: ${top_srcdir}/rmp400.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/rmp400_c_types.h.d

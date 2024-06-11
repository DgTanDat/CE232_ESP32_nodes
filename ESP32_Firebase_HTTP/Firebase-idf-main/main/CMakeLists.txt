# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_ADD_INCLUDEDIRS "")

idf_component_register(
    SRC_DIRS
        "."
        "../components"
    INCLUDE_DIRS
        "../components"
        )

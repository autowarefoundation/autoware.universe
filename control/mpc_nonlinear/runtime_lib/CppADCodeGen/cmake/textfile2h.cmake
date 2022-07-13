include(CMakeParseArguments)

# Function to embed contents of a text file as char array in a C/C++ header file(.h/.hpp).
# The header file will contain a char array and integer variable holding the size of the array.
# Parameters
#   SOURCE_FILE     - The path of source file whose contents will be embedded in the header file.
#   VARIABLE_NAME   - The name of the variable for the byte array. The string "_SIZE" will be append
#                     to this name and will be used a variable name for size variable.
#   HEADER_FILE     - The path of header file.
#   APPEND          - If specified appends to the header file instead of overwriting it
# Usage:
#   text2h(SOURCE_FILE "text.txt" HEADER_FILE "text.h" VARIABLE_NAME "TEXT_TXT")
function(textfile2h)
    set(options APPEND NULL_TERMINATE)
    set(oneValueArgs SOURCE_FILE VARIABLE_NAME HEADER_FILE)
    cmake_parse_arguments(TEXT2H "${options}" "${oneValueArgs}" "" ${ARGN})

    # reads source file contents as hex string
    file(READ "${TEXT2H_SOURCE_FILE}" fileContentString)
    string(LENGTH "${fileContentString}" fileContentStringLength)

    SET(delimiter "*=*")
    STRING(FIND "${fileContentString}" "${delimiter}" pos)
    WHILE(NOT ${pos} LESS 0)
        SET(delimiter "${delimiter}*")
        STRING(FIND "${fileContentString}" "${delimiter}" pos)
    ENDWHILE()

    SET(fileContentString "R\"${delimiter}(${fileContentString})${delimiter}\"")

    # declares char array and the length variables
    set(arrayDefinition "const char ${TEXT2H_VARIABLE_NAME}[] = ${fileContentString};")
    set(stringSizeDefinition "const size_t ${TEXT2H_VARIABLE_NAME}_SIZE = ${fileContentStringLength};")

    set(declarations "${arrayDefinition}\n\n${stringSizeDefinition}\n\n")
    if(TEXT2H_APPEND)
        file(APPEND ${TEXT2H_HEADER_FILE} "${declarations}")
    else()
        file(WRITE ${TEXT2H_HEADER_FILE} "${declarations}")
    endif()
endfunction()
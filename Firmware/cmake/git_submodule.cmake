function(load_git_submodule SUBMODULE_GROUP SUBMODULE_NAME CHECK_SUBFOLDER SET_PATH)
    set(SUBMODULE_RELATIVE_PATH "${SUBMODULE_GROUP}/${SUBMODULE_NAME}")
    set(SUBMODULE_PATH "${STM32_CMAKE_DIR}/${SUBMODULE_RELATIVE_PATH}")
    set(CHECK_FOLDER "${SUBMODULE_PATH}/${CHECK_SUBFOLDER}")
    set(${SET_PATH} "" PARENT_SCOPE) # clear the path

    message("Checking submodule: ${SUBMODULE_PATH}")

    if(EXISTS "${SUBMODULE_PATH}" AND EXISTS "${CHECK_FOLDER}")
        set(${SET_PATH} ${SUBMODULE_PATH} PARENT_SCOPE)
        return()
    endif()

    message("Initializing submodule (if present)")
    execute_process(COMMAND bash -c "git submodule update --init ${SUBMODULE_RELATIVE_PATH}"
                    WORKING_DIRECTORY "${STM32_CMAKE_DIR}"
                    OUTPUT_VARIABLE TMP
                    ERROR_VARIABLE TMP_ERROR
                    OUTPUT_QUIET ERROR_QUIET)
    message("${TMP}")

    if(EXISTS "${SUBMODULE_PATH}" AND EXISTS "${CHECK_FOLDER}")
        set(${SET_PATH} ${SUBMODULE_PATH} PARENT_SCOPE)
        return()
    endif()
endfunction()

function(git_clone GIT_URL SUBMODULE_FOLDER GIT_TAG CHECK_SUBFOLDER SET_PATH)
    get_filename_component(REPO_URL ${GIT_URL} DIRECTORY)
    get_filename_component(REPOSITORY_NAME ${GIT_URL} NAME)
    set(SUBMODULE_RELATIVE_PATH "${SUBMODULE_FOLDER}/${REPOSITORY_NAME}")
    set(SUBMODULE_PATH "${STM32_CMAKE_DIR}/${SUBMODULE_RELATIVE_PATH}")
    set(CHECK_FOLDER "${SUBMODULE_PATH}/${CHECK_SUBFOLDER}")
    set(${SET_PATH} "" PARENT_SCOPE) # clear the path

    if(EXISTS "${SUBMODULE_PATH}" AND EXISTS "${CHECK_FOLDER}")
        if(GIT_TAG)
            message("Checking out version ${GIT_TAG} of module ${SUBMODULE_RELATIVE_PATH}")
            execute_process(COMMAND bash -c "git checkout ${GIT_TAG}"
                            WORKING_DIRECTORY "${STM32_CMAKE_DIR}/${SUBMODULE_RELATIVE_PATH}"
                            OUTPUT_VARIABLE TMP
                            ERROR_VARIABLE TMP_ERROR)
            message("${TMP_ERROR}")

            if(NOT TMP_ERROR MATCHES "error")
                set(${SET_PATH} ${SUBMODULE_PATH} PARENT_SCOPE)
                return()
            else()
                file(REMOVE_RECURSE "${STM32_CMAKE_DIR}/${SUBMODULE_RELATIVE_PATH}")
            endif()
        else()
            set(${SET_PATH} ${SUBMODULE_PATH} PARENT_SCOPE)
            return()
        endif()
    endif()

    if(EXISTS "${SUBMODULE_PATH}")
        message(FATAL_ERROR "Can not clone since folder already exists. Please remove: ${SUBMODULE_PATH}")
        return()
    endif()

    if(GIT_TAG)
        message("Cloning module: ${SUBMODULE_RELATIVE_PATH} # ${GIT_TAG} from https://${GIT_URL}.git")
        execute_process(
            COMMAND
                bash -c
                "git clone --depth 1 --branch ${GIT_TAG} --single-branch https://${GIT_URL}.git ${SUBMODULE_RELATIVE_PATH}"
            WORKING_DIRECTORY "${STM32_CMAKE_DIR}"
            OUTPUT_VARIABLE TMP
            ERROR_VARIABLE TMP_ERROR
            OUTPUT_QUIET ERROR_QUIET)
    else()
        message("Cloning module: ${SUBMODULE_RELATIVE_PATH} from https://${GIT_URL}.git")
        execute_process(COMMAND bash -c "git clone --depth 1 https://${GIT_URL}.git ${SUBMODULE_RELATIVE_PATH}"
                        WORKING_DIRECTORY "${STM32_CMAKE_DIR}"
                        OUTPUT_VARIABLE TMP
                        ERROR_VARIABLE TMP_ERROR
                        OUTPUT_QUIET ERROR_QUIET)
    endif()
    message("${TMP}")

    if(EXISTS "${SUBMODULE_PATH}" AND EXISTS "${CHECK_FOLDER}")
        set(${SET_PATH} ${SUBMODULE_PATH} PARENT_SCOPE)
        return()
    endif()

    if(GIT_TAG)
        message(
            FATAL_ERROR
                "Could not clone or identify the module (${SUBMODULE_RELATIVE_PATH}). Are you connected to the internet? Is the Git tag (${GIT_TAG}) correct?"
            )
    else()
        message(
            FATAL_ERROR
                "Could not clone or identify the module (${SUBMODULE_RELATIVE_PATH}). Are you connected to the internet?"
            )
    endif()
endfunction()

function(git_clone_st SUBMODULE_FOLDER REPOSITORY_NAME GIT_TAG CHECK_SUBFOLDER SET_PATH)
    git_clone("github.com/STMicroelectronics/${REPOSITORY_NAME}"
              ${SUBMODULE_FOLDER}
              "${GIT_TAG}"
              ${CHECK_SUBFOLDER}
              SET_PATH_OUT)
    set(${SET_PATH} ${SET_PATH_OUT} PARENT_SCOPE)
endfunction()

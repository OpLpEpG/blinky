# blinky
umdom 

file stm32f1xx_ll.cmake
```
macro(add_stm32_ll)

    if (NOT STM32_LL_PATH)
        set(STM32_LL_PATH $ENV{ZEPHYR_BASE}/../modules/hal/stm32/stm32cube/stm32f1xx/drivers/src)
    endif()

    foreach(item ${ARGV})

        set(tmp STM32_LL_${item})
        message("add_stm32_ll item <${tmp}|<${${tmp}}>>")
        if ( NOT ${tmp}) 
            message("IF NOT add_stm32_ll item ${tmp} <${${tmp}}>")
            zephyr_library_sources(${STM32_LL_PATH}/stm32f1xx_ll_${item}.c)
            set(${tmp} ${STM32_LL_PATH}/stm32f1xx_ll_${item}.c PARENT_SCOPE)
            message("set ${tmp} ${${tmp}}")
        endif()
        unset(tmp)    
    endforeach()
endmacro(add_stm32_ll)
```


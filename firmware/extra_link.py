# 裸机构建补充: 链接脚本 + newlib-nano/nosys 运行时规格
Import("env")

env.Replace(LDSCRIPT_PATH="ld/STM32F103VC_FLASH.ld")
env.Append(
    LINKFLAGS=[
        "--specs=nano.specs",
        "--specs=nosys.specs",
    ]
)

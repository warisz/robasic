// file system
const modules = [
    { id: 1, name: "Module 1: PID Basics", content_file: "drone/1_pid_intro.txt" },
    { id: 2, name: "Module 2", content_file: "drone/1_pid_intro.txt" },
    { id: 3, name: "Module 3", content_file: "drone/1_pid_intro.txt" },
    { id: 4, name: "Module 4", content_file: "drone/1_pid_intro.txt" },
  ]
  
const sidebar = document.getElementById("sidebar")
const toggleSidebarBtn = document.getElementById("toggleSidebar")
const closeSidebarBtn = document.getElementById("closeSidebar")
const moduleList = document.getElementById("moduleList")
const moduleName = document.getElementById("moduleName")
const moduleText = document.getElementById("moduleText")

async function loadModuleContent(filename) {
    const response = await fetch(`./examples/modules/${filename}`);
    if (!response.ok) {
        throw new Error(`Failed to load ./examples/modules/${filename}`);
    }
    return await response.text();
}
  
function toggleSidebar() {
    sidebar.classList.toggle("open")
}
  
function closeSidebar() {
    sidebar.classList.remove("open")
}
  
async function selectModule(module) {
    moduleName.textContent = module.name
    moduleText.innerHTML = await loadModuleContent(module.content_file)
    closeSidebar()
}
  
toggleSidebarBtn.addEventListener("click", toggleSidebar)
closeSidebarBtn.addEventListener("click", closeSidebar)

modules.forEach((module) => {
    const li = document.createElement("li")
    const button = document.createElement("button")
    button.textContent = module.name
    button.addEventListener("click", () => selectModule(module))
    li.appendChild(button)
    moduleList.appendChild(li)
})

// Initialize with the first module
selectModule(modules[0])
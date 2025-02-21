// file system
const modules = [
    { id: 1, name: "Module 1: PID Basics", content_file: "drone/1_pid_intro.txt", starter_code:
        `# Control the drone's height
current_height = getHeight()

######### TODO: Change the params below!
# PID Controller
k_p = 2.5
k_i = 0.1
k_d = 2.5
#########

setPID(k_p, k_i, k_d);
target_height = 5.0
setHeight(target_height);
`
    },
    { id: 2, name: "Module 2: Motor Mixing", content_file: "drone/2_motor_mixing.txt", starter_code:
        'YUH'
    },
    { id: 3, name: "Module 3", content_file: "drone/1_pid_intro.txt", starter_code:
        'YUH' },
    { id: 4, name: "Module 4", content_file: "drone/1_pid_intro.txt", starter_code:
        'YUH'},
  ]

const sidebar = document.getElementById("sidebar")
const toggleSidebarBtn = document.getElementById("toggleSidebar")
const closeSidebarBtn = document.getElementById("closeSidebar")
const moduleList = document.getElementById("moduleList")
const moduleName = document.getElementById("moduleName")
const moduleText = document.getElementById("moduleText")

let editor = null;
let resolveEditorFunction = null;

let editorReadyPromise = new Promise((resolve) => {
    resolveEditorFunction = resolve;
})

function initializeModules(editorInstance){
    editor = editorInstance;
    console.log("Editor initialized: ", editor);
    resolveEditorFunction(editor); // we are calling the resolve function from the promise so that it saves to editorReadyPromise
}

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
    await editorReadyPromise;
    moduleName.textContent = module.name
    moduleText.innerHTML = await loadModuleContent(module.content_file)
    editor.setValue(module.starter_code)
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
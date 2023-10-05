--[[
=====================================================================
==================== READ THIS BEFORE CONTINUING ====================
=====================================================================
  ros2-with-neovim is heavily inspired from Kickstart.nvim.

  I have basically taken the same approach of having a single file where all the
  main configurations are done in a single file commenting clearly what all of them
  do.

  You should consider this as a starting point into your Neovim journey. Then, you can
  start to add as many plugins and configurations as you see fit.
--]]
-- Set <space> as the leader key
-- See `:help mapleader`
--  NOTE: Must happen before plugins are required (otherwise wrong leader will be used)
vim.g.mapleader = ' '
vim.g.maplocalleader = ' '

-- Install package manager
--    https://github.com/folke/lazy.nvim
--    `:help lazy.nvim.txt` for more info
-- INFO: There are different plugins manager for neovim. I prefer Lazy
local lazypath = vim.fn.stdpath 'data' .. '/lazy/lazy.nvim'
if not vim.loop.fs_stat(lazypath) then
  vim.fn.system {
    'git',
    'clone',
    '--filter=blob:none',
    'https://github.com/folke/lazy.nvim.git',
    '--branch=stable', -- latest stable release
    lazypath,
  }
end

vim.opt.rtp:prepend(lazypath)

-- NOTE: Here is where you install your plugins.
--  You can configure plugins using the `config` key.
--  You can also configure plugins after the setup call,
--    as they will be available in your neovim runtime.
require('lazy').setup({
  -- INFO: Git related plugins
  'tpope/vim-fugitive',        -- Git wrapper for vim
  'rhysd/conflict-marker.vim', -- weapon to fight against merge conflicts
  'rhysd/git-messenger.vim',   -- Shows commit message under cursor
  {
    'lewis6991/gitsigns.nvim', -- Similar to fugitive, but adds additiona functionality
    event = "VeryLazy",
  },
  {
    'sindrets/diffview.nvim', -- Single tabpage interface for easily cycling through diffs
    event = "VeryLazy",
    dependencies = {
      'nvim-lua/plenary.nvim',
    }
  },

  -- INFO: Enhance Editor Experience
  'mg979/vim-visual-multi',      -- Enable multicursor
  'luochen1990/rainbow',         -- Add rainbow color for parenthesis
  'nvim-tree/nvim-web-devicons', -- Add fancy icons
  'skywind3000/asyncrun.vim',    -- Makes the makeprg to run asynchronously
  {
    'nvim-tree/nvim-tree.lua',   -- File tree
    opts = {}
  },
  {
    "folke/todo-comments.nvim", -- Fancy TODOs/FIXMEs
    dependencies = "nvim-lua/plenary.nvim",
    opts = {}
  },
  {
    'ggandor/leap.nvim', -- Improve navigation in file
    config = function()
      require('leap').add_default_mappings()
    end,
  },
  {
    "folke/trouble.nvim", -- Quickfix list for LSP errors
    dependencies = "nvim-tree/nvim-web-devicons",
    opts = {}
  },
  {
    "folke/which-key.nvim", -- Popup with possible keybindings of the command you started to type
    opts = {}
  },
  {
    'nvim-lualine/lualine.nvim', -- Fancier statusline
    opts = {}
  },
  {
    "numToStr/Comment.nvim", -- Comment stuff, everywhere
    opts = {}
  },
  {
    'nvim-pack/nvim-spectre', -- Advance Search and Replace
    opts = {}
  },
  {
    "norcalli/nvim-colorizer.lua", -- Color highlighter
    lazy = true,
    opts = {}
  },
  {
    'romgrk/barbar.nvim', -- Tabline plugin that improves buffers and tabs
    event = "BufEnter",
    dependencies = "nvim-tree/nvim-web-devicons",
    init = function() vim.g.barbar_auto_setup = false end,
    opts = {},
    lazy = true,
  },
  {
    -- Add indentation guides even on blank lines
    'lukas-reineke/indent-blankline.nvim',
    -- Enable `lukas-reineke/indent-blankline.nvim`
    -- See `:help indent_blankline.txt`
    main = "ibl",
    opts = {},
  },
  {
    "akinsho/toggleterm.nvim", -- Improve handling neovim terminals
    opts = {}
  },
  {
    'navarasu/onedark.nvim',
    priority = 1000,
    config = function()
      vim.cmd.colorscheme 'onedark'
    end,
  },
  -- Fuzzy Finder (files, lsp, etc)
  {
    'nvim-telescope/telescope.nvim',
    branch = '0.1.x',
    dependencies = {
      'nvim-lua/plenary.nvim',
      -- Fuzzy Finder Algorithm which requires local dependencies to be built.
      -- Only load if `make` is available. Make sure you have the system
      -- requirements installed.
      {
        'nvim-telescope/telescope-fzf-native.nvim',
        -- NOTE: If you are having trouble with this installation,
        --       refer to the README for telescope-fzf-native for more instructions.
        build = 'make',
        cond = function()
          return vim.fn.executable 'make' == 1
        end,
      },
      "benfowler/telescope-luasnip.nvim", -- Allows to search the available snippet
    },
  },

  --
  -- NOTE: This is where your plugins related to LSP can be installed.
  --  The configuration is done below. Search for lspconfig to find it below.
  {
    -- LSP Configuration & Plugins
    'neovim/nvim-lspconfig',
    dependencies = {
      -- Automatically install LSPs to stdpath for neovim
      { 'williamboman/mason.nvim', config = true },
      'williamboman/mason-lspconfig.nvim',
      'WhoIsSethDaniel/mason-tool-installer.nvim',

      -- Useful status updates for LSP
      -- NOTE: `opts = {}` is the same as calling `require('fidget').setup({})`
      { 'j-hui/fidget.nvim',       tag = 'legacy', opts = {} },

      -- Additional lua configuration, makes nvim stuff amazing!
      'folke/neodev.nvim',
    },
  },
  {
    "jose-elias-alvarez/null-ls.nvim", -- Useful plugin for configuring linter and formatters
    event = "VeryLazy",
    config = function()
      local null_ls = require("null-ls")

      local sources = {
        -- Python
        null_ls.builtins.formatting.black,
        null_ls.builtins.formatting.ruff,
        null_ls.builtins.diagnostics.ruff,

        -- C++
        null_ls.builtins.diagnostics.cpplint.with({
          args = { "--linelength=100", "--filter=-whitespace/braces", "$FILENAME", },
        }),

        null_ls.builtins.formatting.prettier,
        null_ls.builtins.formatting.xmlformat,
      }

      null_ls.setup({
        sources = sources,
        autostart = true,
        default_timeout = 10000,
      })
    end
  },
  {
    -- Autocompletion
    'hrsh7th/nvim-cmp',
    dependencies = {
      -- Snippet Engine & its associated nvim-cmp source
      'L3MON4D3/LuaSnip',
      'saadparwaiz1/cmp_luasnip',

      'hrsh7th/cmp-nvim-lsp', -- Adds LSP completion capabilities

      'hrsh7th/cmp-path',     -- Add source filesystem path
    },
  },
  {
    'rafamadriz/friendly-snippets', -- Snippets collection
    config = function()
      require("luasnip.loaders.from_vscode").lazy_load()

      require("luasnip/loaders/from_vscode").lazy_load({
        paths = {
          vim.fn.stdpath('config') .. '/snippets',
        }
      })
    end,
  },
  {
    -- Highlight, edit, and navigate code
    'nvim-treesitter/nvim-treesitter',
    dependencies = {
      'nvim-treesitter/nvim-treesitter-textobjects',
    },
    build = ':TSUpdate',
  },
  -- INFO: Debug adapters
  'mfussenegger/nvim-dap-python',    -- Python debug adapter
  'rcarriga/nvim-dap-ui',            -- UI-like for debugging
  'theHamsta/nvim-dap-virtual-text', -- Inline text during debugging
  'mfussenegger/nvim-dap',           -- Enable debug adapters

  -- NOTE: Here you can add additional plugins that can enhance your Neovim Journey
}, {})

--[[INFO: Editor Settingss
 --   Take the time to check the settings underneath and configure them base on your needs and preferences
 --]]
-- Set highlight on search
vim.o.hlsearch = false

-- Make line numbers default
vim.opt.nu = true
vim.opt.relativenumber = true

-- Enable mouse mode
vim.o.mouse = 'a'

-- Enable break indent
vim.o.breakindent = true

-- Save undo history
vim.o.undofile = true

-- Case insensitive searching UNLESS /C or capital in search
vim.o.ignorecase = true
vim.o.smartcase = true

-- Decrease update time
vim.o.updatetime = 250
vim.o.timeoutlen = 300

-- Keep signcolumn on
vim.wo.signcolumn = 'yes'
vim.opt.signcolumn = 'auto:4' -- Enable expanding signcolumn

-- Highlight current line
vim.opt.cursorline = true

-- Render the column delimiter
vim.opt.colorcolumn = '100'

-- Prefer spaces of 2 over tabs
vim.opt.tabstop = 2
vim.opt.shiftwidth = 2
vim.opt.expandtab = true

-- Render trailing spaces
vim.opt.listchars = { trail = '·', eol = '↵', tab = '↦↦' }

-- Share system clipboard
vim.opt.clipboard = "unnamedplus"

-- Set completeopt to have a better completion experience
vim.o.completeopt = 'menuone,noselect'

-- Remap for dealing with word wrap
vim.keymap.set('n', 'k', "v:count == 0 ? 'gk' : 'k'", { expr = true, silent = true })
vim.keymap.set('n', 'j', "v:count == 0 ? 'gj' : 'j'", { expr = true, silent = true })

-- set termguicolors to enable highlight groups
vim.opt.termguicolors = true

--  Highlight on yank
local highlight_group = vim.api.nvim_create_augroup('YankHighlight', { clear = true })
vim.api.nvim_create_autocmd('TextYankPost', {
  callback = function()
    vim.highlight.on_yank()
  end,
  group = highlight_group,
  pattern = '*',
})

-- Relative line numbers
local numtogGrp = vim.api.nvim_create_augroup("NumberToggle", { clear = true })
vim.api.nvim_create_autocmd(
  { "BufEnter", "InsertLeave", "FocusGained" },
  {
    pattern = "*",
    callback = function()
      vim.opt.relativenumber = true
    end,
    group = numtogGrp,
    desc = "Turn on relative line numbering when the buffer is entered.",
  }
)
vim.api.nvim_create_autocmd(
  { "BufLeave", "InsertEnter", "FocusLost" },
  {
    pattern = "*",
    callback = function()
      vim.opt.relativenumber = false
    end,
    group = numtogGrp,
    desc = "Turn off relative line numbering when the buffer is exited.",
  }
)

-- Syntax highlighting for specific filetypes
vim.api.nvim_create_autocmd(
  { "BufRead", "BufNewFile" },
  { pattern = "*.xacro", command = "set filetype=xml" }
)
vim.api.nvim_create_autocmd(
  { "BufRead", "BufNewFile" },
  { pattern = "*.launch", command = "set filetype=xml" }
)


--[[INFO: Lualine
 --   This configuration can be used to add useful information to the statusbar underneath neovim
 --]]

local function get_venv()
  local venv = vim.env.VIRTUAL_ENV
  if venv then
    local env = string.match(venv, "[^/]+$")
    return ' ' .. env
  else
    return ''
  end
end

local ros_distro = vim.fn.expand('$ROS_DISTRO')

local function get_ros_distro()
  if ros_distro and ros_distro ~= '$ROS_DISTRO' then
    return '󰭆 ' .. ros_distro
  else
    return ''
  end
end

-- Set lualine as statusline
-- See `:help lualine.txt`
require('lualine').setup {
  options = {
    icons_enabled = true,
    theme = 'onedark',
    component_separators = '|',
    section_separators = '',
    ignore_focus = {
      "dapui_watches", "dapui_breakpoints",
      "dapui_scopes", "dapui_console",
      "dapui_stacks", "dap-repl"
    },
    disabled_filetypes = { 'NvimTree' }
  },
  sections = {
    lualine_a = { 'mode' },
    lualine_b = { 'branch', 'diff', 'diagnostics' },
    lualine_c = { 'filename' },
    -- lualine_x = { { get_venv }, { get_ros_distro }, 'fileformat', 'filetype' },
    lualine_x = { { get_venv }, { get_ros_distro }, 'fileformat', 'filetype' },
    lualine_y = { 'progress' },
    lualine_z = { 'location' }
  },
}

--[[INFO: Telescope
 --   Configure telescope
 --]]
local ros_distro = vim.fn.expand('$ROS_DISTRO')
require('telescope').setup {
  pickers = {
    find_files = {
      find_command = { "rg", "--files", "--hidden", "-g", "!.git", "--ignore-file",
        vim.fn.expand("      $HOME/.config/nvim/.rignore") },
    },
    lsp_document_symbols = {
      show_line = true
    }
  },
  extensions = {
    fzf = {
      fuzzy = true,                   -- false will only do exact matching
      override_generic_sorter = true, -- override the generic sorter
      override_file_sorter = true,    -- override the file sorter
      case_mode = "smart_case",       -- or "ignore_case" or "respect_case"
      -- the default case_mode is "smart_case"
    }
  },
}
require('telescope').load_extension('fzf')
require('telescope').load_extension('luasnip')

--[[ INFO: Treesitter ]]
vim.defer_fn(function()
  require('nvim-treesitter.configs').setup {
    -- Add languages to be installed here that you want installed for treesitter
    ensure_installed = {
      'bash',
      'c',
      'cpp',
      'csv',
      'dockerfile',
      'gitcommit',
      'gitignore',
      'html',
      'json',
      'lua',
      'markdown',
      'python',
      'typescript',
      'vim',
      'xml',
      'yaml'
    },

    -- Autoinstall languages that are not installed. Defaults to false (but you can change for yourself!)
    auto_install = false,
    highlight = { enable = true },
    indent = { enable = true, disable = { 'python', 'cpp' } },
    incremental_selection = {
      enable = true,
      keymaps = {
        init_selection = '<c-space>',
        node_incremental = '<c-space>',
        scope_incremental = '<c-s>',
        node_decremental = '<c-backspace>',
      },
    },
    textobjects = {
      select = {
        enable = true,
        lookahead = true, -- Automatically jump forward to textobj, similar to targets.vim
        keymaps = {
          -- You can use the capture groups defined in textobjects.scm
          ['aa'] = '@parameter.outer',
          ['ia'] = '@parameter.inner',
          ['af'] = '@function.outer',
          ['if'] = '@function.inner',
          ['ac'] = '@class.outer',
          ['ic'] = '@class.inner',
        },
      },
      move = {
        enable = true,
        set_jumps = true, -- whether to set jumps in the jumplist
        goto_next_start = {

          [']]'] = '@class.outer',
        },
        goto_next_end = {
          [']M'] = '@function.outer',
          [']['] = '@class.outer',
        },
        goto_previous_start = {
          ['[m'] = '@function.outer',
          ['[['] = '@class.outer',
        },
        goto_previous_end = {
          ['[M'] = '@function.outer',
          ['[]'] = '@class.outer',
        },
      },
      swap = {
        enable = true,
        swap_next = {
          ['<leader>a'] = '@parameter.inner',
        },
        swap_previous = {
          ['<leader>A'] = '@parameter.inner',
        },
      },
    },
  }
end, 0)

--[[ INFO: Toggleterm configuration]]
require("toggleterm").setup({
  size = 20,
  open_mapping = [[<c-\>]],
  hide_numbers = true,
  shade_filetypes = {},
  shade_terminals = true,
  shading_factor = 2,
  start_in_insert = true,
  insert_mappings = true,
  persist_size = true,
  direction = "float",
  close_on_exit = true,
  shell = vim.o.shell,
  float_opts = {
    border = "curved",
    winblend = 0,
    highlights = {
      border = "Normal",
      background = "Normal",
    },
  },
})

function _G.set_terminal_keymaps()
  local opts = { noremap = true }
  vim.api.nvim_buf_set_keymap(0, 't', '<esc>', [[<C-\><C-n>]], opts)
  vim.api.nvim_buf_set_keymap(0, 't', '<C-h>', [[<C-\><C-n><C-W>h]], opts)
  vim.api.nvim_buf_set_keymap(0, 't', '<C-j>', [[<C-\><C-n><C-W>j]], opts)
  vim.api.nvim_buf_set_keymap(0, 't', '<C-k>', [[<C-\><C-n><C-W>k]], opts)
  vim.api.nvim_buf_set_keymap(0, 't', '<C-l>', [[<C-\><C-n><C-W>l]], opts)
end

--[[ INFO: LSP Configuration ]]
local on_attach = function(_, bufnr)
  -- NOTE: Remember that lua is a real programming language, and as such it is possible
  -- to define small helper and utility functions so you don't have to repeat yourself
  -- many times.
  --
  -- In this case, we create a function that lets us more easily define mappings specific
  -- for LSP related items. It sets the mode, buffer and description for us each time.
  local nmap = function(keys, func, desc)
    if desc then
      desc = 'LSP: ' .. desc
    end

    vim.keymap.set('n', keys, func, { buffer = bufnr, desc = desc })
  end

  nmap('<leader>rn', vim.lsp.buf.rename, '[R]e[n]ame')
  nmap('<leader>ca', vim.lsp.buf.code_action, '[C]ode [A]ction')

  nmap('gd', vim.lsp.buf.definition, '[G]oto [D]efinition')
  -- nmap('gI', vim.lsp.buf.implementation, '[G]oto [I]mplementation')
  nmap('<leader>D', vim.lsp.buf.type_definition, 'Type [D]efinition')

  -- See `:help K` for why this keymap
  nmap('K', vim.lsp.buf.hover, 'Hover Documentation')
  nmap('<C-k>', vim.lsp.buf.signature_help, 'Signature Documentation')

  -- Lesser used LSP functionality
  nmap('gD', vim.lsp.buf.declaration, '[G]oto [D]eclaration')
  nmap('<leader>wa', vim.lsp.buf.add_workspace_folder, '[W]orkspace [A]dd Folder')
  nmap('<leader>wr', vim.lsp.buf.remove_workspace_folder, '[W]orkspace [R]emove Folder')
  nmap('<leader>wl', function()
    print(vim.inspect(vim.lsp.buf.list_workspace_folders()))
  end, '[W]orkspace [L]ist Folders')

  -- Create a command `:Format` local to the LSP buffer
  vim.api.nvim_buf_create_user_command(bufnr, 'Format', function(_)
    vim.lsp.buf.format()
  end, { desc = 'Format cursent buffer with LSP' })

  nmap('<S-Tab>', ':ClangdSwitchSourceHeader<CR>', 'Switch between header and source files')
end

-- Enable the following language servers
--  Feel free to add/remove any LSPs that you want here. They will automatically be installed.
--
--  Add any additional override configuration in the following tables. They will be passed to
--  the `settings` field of the server config. You must look up that documentation yourself.
local servers = {
  bashls = {},
  clangd = {
    cmd = {
      -- see clangd --help-hidden
      "clangd",
      "--background-index",
      -- by default, clang-tidy use -checks=clang-diagnostic-*,clang-analyzer-*
      -- to add more checks, create .clang-tidy file in the root directory
      -- and add Checks key, see https://clang.llvm.org/extra/clang-tidy/
      "--clang-tidy",
      "--completion-style=bundled",
      "--cross-file-rename",
      "--header-insertion=iwyu",
    },
  },
  cmake = {},
  cssls = {},
  -- gopls = {},
  -- Change diagnostic mode due to an error where it starts to scan everything if no pyproject.toml is found
  -- https://www.reddit.com/r/neovim/comments/135fqp9/why_is_pyright_constantly_analyzing_files_it/
  pyright = {
    python = {
      analysis = {
        autoSearchPaths = true,
        -- diagnosticMode = "workspace",
        diagnosticMode = "openFilesOnly",
        useLibraryCodeForTypes = true,
        reportDuplicateImport = true
      }
    }
  },

  lua_ls = {
    Lua = {
      workspace = { checkThirdParty = false },
      telemetry = { enable = false },
      diagnostics = { globals = { 'vim', 'require' } },
    },
  },
  tsserver = {}
}

-- Setup neovim lua configuration
require('neodev').setup()

-- nvim-cmp supports additional completion capabilities, so broadcast that to servers
local capabilities = vim.lsp.protocol.make_client_capabilities()
capabilities = require('cmp_nvim_lsp').default_capabilities(capabilities)
capabilities.offsetEncoding = { "utf-16" }
capabilities.textDocument.completion.completionItem.snippetSupport = true

-- Setup mason so it can manage external tooling
require('mason').setup()

-- Ensure the servers above are installedpair
local mason_lspconfig = require 'mason-lspconfig'

mason_lspconfig.setup {
  ensure_installed = vim.tbl_keys(servers),
}

require('mason-tool-installer').setup {

  -- List of all DAP, Linter and Formatters to install
  ensure_installed = {
    -- DAP
    "codelldb",
    "cpptools",

    -- Linter
    "cpplint",

    -- Formatter
    "prettier",
    "xmlformatter",
  },

  -- if set to true this will check each tool for updates. If updates
  -- are available the tool will be updated. This setting does not
  -- affect :MasonToolsUpdate or :MasonToolsInstall.
  -- Default: false
  auto_update = false,

  -- automatically install / update on startup. If set to false nothing
  -- will happen on startup. You can use :MasonToolsInstall or
  -- :MasonToolsUpdate to install tools and check for updates.
  -- Default: true
  run_on_start = true,

  -- set a delay (in ms) before the installation starts. This is only
  -- effective if run_on_start is set to true.
  -- e.g.: 5000 = 5 second delay, 10000 = 10 second delay, etc...
  -- Default: 0
  start_delay = 3000, -- 3 second delay

  -- Only attempt to install if 'debounce_hours' number of hours has
  -- elapsed since the last time Neovim was started. This stores a
  -- timestamp in a file named stdpath('data')/mason-tool-installer-debounce.
  -- This is only relevant when you are using 'run_on_start'. It has no
  -- effect when running manually via ':MasonToolsInstall' etc....
  -- Default: nil
  debounce_hours = 5, -- at least 5 hours between attempts to install/update
}

mason_lspconfig.setup_handlers {
  function(server_name)
    require('lspconfig')[server_name].setup {
      capabilities = capabilities,
      on_attach = on_attach,
      settings = servers[server_name],
    }
  end,
}

-- nvim-cmp setup
local cmp = require 'cmp'
local luasnip = require 'luasnip'

cmp.setup {
  snippet = {
    expand = function(args)
      luasnip.lsp_expand(args.body)
    end,
  },
  mapping = cmp.mapping.preset.insert {
    ['<C-n>'] = cmp.mapping.select_next_item(),
    ['<C-p>'] = cmp.mapping.select_prev_item(),
    ['<C-d>'] = cmp.mapping.scroll_docs(-4),
    ['<C-f>'] = cmp.mapping.scroll_docs(4),
    ['<C-Space>'] = cmp.mapping.complete(),
    ['<CR>'] = cmp.mapping.confirm {
      behavior = cmp.ConfirmBehavior.Replace,
      select = true,
    },
    ['<Tab>'] = cmp.mapping(function(fallback)
      if cmp.visible() then
        cmp.select_next_item()
      elseif luasnip.expand_or_jumpable() then
        luasnip.expand_or_jump()
      else
        fallback()
      end
    end, { 'i', 's' }),
    ['<S-Tab>'] = cmp.mapping(function(fallback)
      if cmp.visible() then
        cmp.select_prev_item()
      elseif luasnip.jumpable(-1) then
        luasnip.jump(-1)
      else
        fallback()
      end
    end, { 'i', 's' }),
  },
  sources = {
    { name = 'nvim_lsp' },
    { name = 'luasnip' },
    { name = 'path' },
  },
}

-- Diagnostic signs
-- NOTE: If the symbols don't show you would need a nerdfont. Otherwise, you can simply remove the custom symbols
local diagnostic_signs = {
  { name = "DiagnosticSignError", text = "" },
  { name = "DiagnosticSignWarn", text = "" },
  { name = "DiagnosticSignHint", text = "" },
  { name = "DiagnosticSignInfo", text = "" },
}
for _, sign in ipairs(diagnostic_signs) do
  vim.fn.sign_define(sign.name, { texthl = sign.name, text = sign.text, numhl = sign.name })
end

vim.diagnostic.config({
  virtual_text = {
    -- source = "always",  -- Or "if_many"
    prefix = '●', -- Could be '■', '▎', 'x'
  },
  severity_sort = true,
  float = {
    source = "always", -- Or "if_many"
  },
  signs = true
})

-- Suggested approach to cancel snippet session after going back to normal mode
-- Taken from https://github.com/L3MON4D3/LuaSnip/issues/258#issuecomment-1011938524
function leave_snippet()
  if
      ((vim.v.event.old_mode == 's' and vim.v.event.new_mode == 'n') or vim.v.event.old_mode == 'i')
      and require('luasnip').session.current_nodes[vim.api.nvim_get_current_buf()]
      and not require('luasnip').session.jump_active
  then
    require('luasnip').unlink_current()
  end
end

-- stop snippets when you leave to normal mode
vim.api.nvim_command([[
    autocmd ModeChanged * lua leave_snippet()
]])

-- Custom command to disable completion
-- Taken from https://gist.github.com/bnse/a3eb5b9941e6af4582c5406b29d76e05
local cmp_enabled = true
vim.api.nvim_create_user_command("ToggleAutoComplete", function()
  if cmp_enabled then
    require("cmp").setup.buffer({ enabled = false })
    cmp_enabled = false
  else
    require("cmp").setup.buffer({ enabled = true })
    cmp_enabled = true
  end
end, {})

--[[ INFO: Debug Adapters Configuration ]]
local dap = require('dap')
require("dapui").setup({
  controls = {
    icons = {
      pause = '⏸',
      play = '▶',
      terminate = '⏹',
    },
  },
})
require("nvim-dap-virtual-text").setup()

local sign = vim.fn.sign_define

sign("DapBreakpoint", { text = "●", texthl = "DapBreakpoint", linehl = "", numhl = "" })
sign("DapBreakpointCondition", { text = "●", texthl = "DapBreakpointCondition", linehl = "", numhl = "" })
sign("DapLogPoint", { text = "◆", texthl = "DapLogPoint", linehl = "", numhl = "" })

-- C++
dap.adapters.codelldb = {
  type = 'server',
  port = "${port}",
  executable = {
    command = vim.fn.expand('$HOME/.local/share/nvim/mason/bin/codelldb'),
    args = { "--port", "${port}" },
  }
}
dap.configurations.cpp = {
  {
    name = "C++: Run file",
    type = "codelldb",
    request = "launch",
    program = function()
      return vim.fn.input('Path to executable: ', vim.fn.getcwd() .. '/', 'file')
    end,
    cwd = '${workspaceFolder}',
    stopOnEntry = false,
  },
  {
    -- If you get an "Operation not permitted" error using this, try disabling YAMA:
    --  echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope
    name = "C++: Attach to process",
    type = 'codelldb', -- Adjust this to match your adapter name (`dap.adapters.<name>`)
    request = 'attach',
    pid = require('dap.utils').pick_process,
    args = {},
  },
  {
    name = "C++: ROS Node",
    type = "codelldb",
    request = "launch",
    program = function()
      local pkgName = vim.fn.input('ROS Package: ', 'demo_nodes_cpp')
      return vim.fn.input('Path to executable: ', vim.fn.getcwd() .. '/install/' .. pkgName .. '/lib/' .. pkgName .. '/',
        'file')
    end,
    cwd = '${workspaceFolder}',
    stopOnEntry = false,
  },

}
-- C++
require('dap-python').setup()
table.insert(require('dap').configurations.python, {
  type = 'python',
  request = 'launch',
  name = 'Python: ROS2 lauch test',
  program = '/opt/ros/humble/bin/launch_test',
  args = { "${file}" },
})
require('dap-python').test_runner = 'pytest'

--[[ INFO: Nvim Colorizer configuration
-- WARN: It must be done after configuring color scheme
--]]
require 'colorizer'.setup {
  'css',
  'javascript',
  'lua',
  'html',
}

--[[ INFO: Keymaps configurations
--    Make sure to change these keymaps so that they make the most sense to you
--]]

-- Improve motions
vim.keymap.set("n", "<C-d>", "<C-d>zz")
vim.keymap.set("n", "<C-u>", "<C-u>zz")
vim.keymap.set("n", "n", "nzzzv")
vim.keymap.set("n", "N", "Nzzzv")

-- Improve splits navigation
vim.keymap.set("n", "<C-h>", "<C-W>h")
vim.keymap.set("n", "<C-j>", "<C-W>j")
vim.keymap.set("n", "<C-k>", "<C-W>k")
vim.keymap.set("n", "<C-l>", "<C-W>l")

-- Improve pasting
vim.keymap.set("x", "<leader>p", [["_dP]], { desc = "Preserve previous word when pasting" })

-- Diagnostic keymaps
vim.keymap.set("n", "[d", vim.diagnostic.goto_prev)
vim.keymap.set("n", "]d", vim.diagnostic.goto_next)
vim.keymap.set("n", "<leader>e", vim.diagnostic.open_float, { desc = "Open diagnostic in floating window" })
vim.keymap.set("n", "<leader>q", vim.diagnostic.setloclist, { desc = "Send diagnostic to loclist" })
vim.keymap.set("n", "<leader>dd", vim.diagnostic.disable, { desc = "[D]iagnostics [D]disable" })
vim.keymap.set("n", "<leader>de", vim.diagnostic.enable, { desc = "[D]iagnostics [E]nable" })

-- Change workingdir
vim.keymap.set("n", "<leader>cw", ":cd %:p:h<CR>:pwd<CR>", { desc = "Change current workding dir" })
-- Fix forward jump after setting <TAB>
-- https://github.com/neovim/neovim/issues/20126
vim.keymap.set("n", "<C-I>", "<C-I>", { noremap = true })

-- Editor experience
vim.keymap.set("n", "<C-s>", ":write<CR>", { desc = "Save file" })
vim.keymap.set("n", "<leader>cd", ":ToggleAutoComplete<CR>", { desc = "[C]ompletion [D]isable" })
vim.keymap.set("n", "<leader>ce", ":ToggleAutoComplete<CR>", { desc = "[C]ompletion [E]nable" })

-- ====================================================
-- Telescope
-- ====================================================
vim.keymap.set("n", "<leader>?", require("telescope.builtin").oldfiles, { desc = "[?] Find recently opened files" })
vim.keymap.set("n", "<leader><space>", require("telescope.builtin").buffers, { desc = "[ ] Find existing buffers" })
vim.keymap.set("n", "<leader>/", function()
  -- You can pass additional configuration to telescope to change theme, layout, etc.
  require("telescope.builtin").current_buffer_fuzzy_find(require("telescope.themes").get_dropdown {
    previewer = false,
    sorting_strategy = "ascending",
  })
end, { desc = "[/] Fuzzily search in current buffer]" })

vim.keymap.set("n", "<leader>sf", require("telescope.builtin").find_files, { desc = "[S]earch [F]iles" })
vim.keymap.set("n", "<leader>sh", require("telescope.builtin").help_tags, { desc = "[S]earch [H]elp" })
vim.keymap.set("n", "<leader>sw", require("telescope.builtin").grep_string, { desc = "[S]earch current [W]ord" })
vim.keymap.set("n", "<leader>sg", require("telescope.builtin").live_grep, { desc = "[S]earch by [G]rep" })
vim.keymap.set("n", "<leader>sd", require("telescope.builtin").diagnostics, { desc = "[S]earch [D]iagnostics" })
vim.keymap.set("n", "<leader>sp", require("telescope.builtin").spell_suggest, { desc = "[S][P]ell suggestion" })
vim.keymap.set("n", "<leader>sk", require("telescope.builtin").keymaps, { desc = "[S]earch [K]eymaps" })
vim.keymap.set("n", "<leader>gf", require("telescope.builtin").git_files, { desc = "Search [G]it [F]iles" })
vim.keymap.set("n", "<leader>sc", require("telescope.builtin").git_commits, { desc = "[S]earch git [C]ommits" })
vim.keymap.set("n", '<leader>ds', require('telescope.builtin').lsp_document_symbols, { desc = '[D]ocument [S]ymbols' })
vim.keymap.set("n", '<leader>ws', require('telescope.builtin').lsp_dynamic_workspace_symbols,
  { desc = '[W]orkspace [S]ymbols' })
vim.keymap.set("n", "gr", require("telescope.builtin").lsp_references, { desc = "[G]oto [R]eferences" })
vim.keymap.set("n", "gI", require("telescope.builtin").lsp_implementations, { desc = '[G]oto [I]mplementation' })

-- ====================================================
-- Nvim Tree
-- ====================================================
vim.keymap.set("n", "<leader>ff", ":NvimTreeFindFile<CR>", { desc = "NvimTree [F]ind [F]ile" })
vim.keymap.set("n", "<leader>tt", ":NvimTreeToggle<CR>", { desc = "NvimTree [T]ree [T]oggle" })

-- ====================================================
-- Execute over files
-- ====================================================
vim.keymap.set("n", "<leader>ru", ":w<CR>:!%:p", { desc = "[R][U]n current file" })
vim.keymap.set("n", "<leader>me", ":!chmod +x %:p<CR>", { desc = "[M]ake current file [E]xecutable" })

-- ====================================================
-- Spectre
-- ====================================================
vim.keymap.set("n", "<leader>P", require("spectre").open, { desc = "Open Search and Replace" })

-- ====================================================
-- LSP
-- ====================================================
vim.keymap.set("n", "<leader>f", vim.lsp.buf.format)
vim.keymap.set('v', '<Leader>f', vim.lsp.buf.format, { desc = "Format visual selected lines" })
vim.keymap.set("n", "<leader>gc", "<cmd>Neogen<CR>", { desc = "[G]enerate [D]ocstring" })

-- ====================================================
-- Trouble
-- ====================================================
vim.keymap.set("n", "<leader>xx", "<cmd>TroubleToggle<cr>",
  { silent = true, noremap = true, desc = "Toggle trouble window" }
)
vim.keymap.set("n", "<leader>xw", "<cmd>TroubleToggle workspace_diagnostics<cr>",
  { silent = true, noremap = true, desc = "Trouble diagnostics for the whole workspace" }
)
vim.keymap.set("n", "<leader>xd", "<cmd>TroubleToggle document_diagnostics<cr>",
  { silent = true, noremap = true, desc = "Trouble diagnostics for current document" }
)

-- ====================================================
-- Debugging -> dap
-- ====================================================
vim.keymap.set("n", "<F2>", ":lua require('dapui').toggle()<CR>")
vim.keymap.set("n", "<F3>", ":lua require('dap').continue()<CR>")
vim.keymap.set("n", "<F4>", ":lua require('dap').step_over()<CR>")
vim.keymap.set("n", "<F5>", ":lua require('dap').step_into()<CR>")
vim.keymap.set("n", "<F6>", ":lua require('dap').step_out()<CR>")
vim.keymap.set("n", "<F7>", ":DapTerminate<CR>")
vim.keymap.set("n", "<leader>br", ":lua require('dap').toggle_breakpoint()<CR>")
vim.api.nvim_create_user_command("DapPytestMethod", ":lua require('dap-python').test_method()", {})
vim.keymap.set("n", "<leader>dm", ":lua require('dap-python').test_method()<CR>",
  { silent = true, noremap = true, desc = "DapPytest : Debug method" })
vim.keymap.set("n", "<leader>df", ":lua require('dap-python').test_class()<CR>",
  { silent = true, noremap = true, desc = "DapPytest : Debug class" })

-- ====================================================
-- barbar --> Tabs management
-- ====================================================
vim.keymap.set("n", "<A-,>", "<cmd>BufferPrevious<cr>",
  { silent = true, noremap = true, desc = "Go to previous tab" }
)
vim.keymap.set("n", "<A-.>", "<cmd>BufferNext<cr>",
  { silent = true, noremap = true, desc = "Go to next tab" }
)
vim.keymap.set("n", "<A-<>", "<cmd>BufferMovePrevious<cr>",
  { silent = true, noremap = true, desc = "Move tab to the left" }
)
vim.keymap.set("n", "<A->>", "<cmd>BufferMoveNext<cr>",
  { silent = true, noremap = true, desc = "Move tab to the right" }
)
vim.keymap.set("n", "<A->>", "<cmd>BufferMoveNext<cr>",
  { silent = true, noremap = true, desc = "Move tab to the right" }
)
vim.keymap.set("n", "<A-c>", "<cmd>BufferClose<cr>",
  { silent = true, noremap = true, desc = "Close current buffer" }
)
local opts = { noremap = true, silent = true }
-- Goto buffer in position...
vim.keymap.set('n', '<A-1>', '<Cmd>BufferGoto 1<CR>', opts)
vim.keymap.set('n', '<A-2>', '<Cmd>BufferGoto 2<CR>', opts)
vim.keymap.set('n', '<A-3>', '<Cmd>BufferGoto 3<CR>', opts)
vim.keymap.set('n', '<A-4>', '<Cmd>BufferGoto 4<CR>', opts)
vim.keymap.set('n', '<A-5>', '<Cmd>BufferGoto 5<CR>', opts)
vim.keymap.set('n', '<A-6>', '<Cmd>BufferGoto 6<CR>', opts)
vim.keymap.set('n', '<A-7>', '<Cmd>BufferGoto 7<CR>', opts)
vim.keymap.set('n', '<A-8>', '<Cmd>BufferGoto 8<CR>', opts)
vim.keymap.set('n', '<A-9>', '<Cmd>BufferGoto 9<CR>', opts)
vim.keymap.set('n', '<A-0>', '<Cmd>BufferLast<CR>', opts)

-- ====================================================
-- ROS 2 related commands
-- ====================================================
-- Build
vim.api.nvim_command([[
  command! ColconBuild :! CC=clang CXX=clang++ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
]])
vim.api.nvim_command([[
  command! -nargs=1 ColconBuildSingle :! CC=clang CXX=clang++ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-up-to <args>
]])
vim.api.nvim_command([[
  command! ColconBuildDebug :! CC=clang CXX=clang++ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
]])
vim.api.nvim_command([[
  command! -nargs=1 ColconBuildDebugSingle :! CC=clang CXX=clang++ colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug --packages-up-to <args>
]])

-- Test
vim.api.nvim_command([[
  command! ColconTest :! colcon test
]])
vim.api.nvim_command([[
  command! -nargs=1 ColconTestSingle :! colcon test --packages-select <args>
]])
vim.api.nvim_command([[
  command! ColconTestResult :! colcon test-result --all
]])

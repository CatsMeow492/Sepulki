const { Marp } = require('@marp-team/marp-core')
const { writeFileSync } = require('fs')

const marp = new Marp({
  mermaid: true,
  allowLocalFiles: true
})

const { html, css } = marp.render(readFileSync('Pitch/presentation.md', 'utf-8'))

writeFileSync('presentation.html', `<style>${css}</style>${html}`)
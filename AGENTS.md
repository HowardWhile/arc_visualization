# AGENTS.md

## Agent skills

### Issue tracker

議題以本機 Markdown 檔案管理，位於 `.scratch/<feature>/`。詳見 `docs/agents/issue-tracker.md`。

### Triage labels

使用五個標準 triage 角色的預設標籤字串。詳見 `docs/agents/triage-labels.md`。

### Domain docs

本專案採用 single-context 領域文件佈局。詳見 `docs/agents/domain.md`。

<!-- CODEGRAPH_START -->
## CodeGraph

In repositories indexed by CodeGraph (a `.codegraph/` directory exists at the repo root), reach for it BEFORE grep/find or reading files when you need to understand or locate code:

- **MCP tool** (when available): `codegraph_explore` answers most code questions in one call — the relevant symbols' verbatim source plus the call paths between them, including dynamic-dispatch hops grep can't follow. Name a file or symbol in the query to read its current line-numbered source. If it's listed but deferred, load it by name via tool search.
- **Shell** (always works): `codegraph explore "<symbol names or question>"` prints the same output.

If there is no `.codegraph/` directory, skip CodeGraph entirely — indexing is the user's decision.
<!-- CODEGRAPH_END -->

## Git 提交禁止

除非使用者明確同意，不得執行 git stage 更不能自動 git commit。
# Copy of __expand_tilde_by_ref from bash-completion
__python_argcomplete_expand_tilde_by_ref () {
    if [ "${!1:0:1}" = "~" ]; then
        if [ "${!1}" != "${!1//\/}" ]; then
            eval $1="${!1/%\/*}"/'${!1#*/}';
        else
            eval $1="${!1}";
        fi;
    fi
}
# complete -o default -o bashdefault -D -F _python_argcomplete_global
eval "$(register-python-argcomplete boxi)"
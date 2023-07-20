use core::str::FromStr;

use proc_macro2::TokenStream;
use syn::DeriveInput;

// SyncCompGroup
    fn sync_comp_group_impl(ast : DeriveInput, comp_type : TokenStream) -> proc_macro::TokenStream {
        match ast.data {
            // Macro works only on structs
            syn::Data::Struct(data) => {
                // Struct metadata
                let name = ast.ident;
                let fields = data.fields;
                let fields_count = fields.len();

                // Streams for different functions
                let mut setup_s = TokenStream::new();
                let mut for_each_s = TokenStream::new();
                let mut for_each_mut_s = TokenStream::new();
                let mut try_for_each_s = TokenStream::new();
                let mut try_for_each_mut_s = TokenStream::new();
                let mut for_each_dyn_s = TokenStream::new();

                let mut findex : usize  = 0;
                for field in fields {
                    // Name of current field
                    let fname = if let Some(ident) = field.ident {
                        quote::quote! { #ident }
                    } else {
                        TokenStream::from_str(&format!("{}", findex)).unwrap()
                    };  

                    // Extend streams
                        setup_s.extend(quote::quote! {
                            self.#fname.setup()?;
                        });

                        for_each_s.extend(quote::quote! {
                            res[#findex] = func(&self.#fname, #findex);
                        }); 

                        for_each_mut_s.extend(quote::quote! {
                            res[#findex] = func(&mut self.#fname, #findex);
                        }); 

                        try_for_each_s.extend(quote::quote! {
                            res[#findex] = func(&self.#fname, #findex)?;
                        }); 

                        try_for_each_mut_s.extend(quote::quote! {
                            res[#findex] = func(&mut self.#fname, #findex)?;
                        }); 

                        for_each_dyn_s.extend(quote::quote! {
                            res.push(func(&self.#fname, #findex));
                        }); 
                    // 

                    findex += 1;
                }

                quote::quote! {
                    impl syact::Setup for #name {
                        fn setup(&mut self) -> Result<(), syact::Error> {
                            #setup_s
                            Ok(())
                        }
                    }

                    impl syact::comp::group::SyncCompGroup<#comp_type, #fields_count> for #name { 
                        fn for_each<'a, F, R>(&'a self, mut func : F) -> [R; #fields_count]
                        where 
                            F : FnMut(&'a Self::Comp, usize) -> R,
                            R : Copy + Default 
                        {   
                            let mut res = [R::default(); #fields_count];
                            #for_each_s             // Insert stream
                            res
                        }
                    
                        fn for_each_mut<F, R>(&mut self, mut func : F) -> [R; #fields_count]
                        where 
                            F : FnMut(&mut Self::Comp, usize) -> R,
                            R : Copy + Default 
                        {
                            let mut res = [R::default(); #fields_count];
                            #for_each_mut_s         // Insert stream
                            res
                        }
                    
                        fn try_for_each<'a, F, R, E>(&'a self, mut func : F) -> Result<[R; #fields_count], E>
                        where 
                            F : FnMut(&'a Self::Comp, usize) -> Result<R, E>,
                            R : Copy + Default 
                        {
                            let mut res = [R::default(); #fields_count];
                            #try_for_each_s         // Insert stream
                            Ok(res)
                        }
                    
                        fn try_for_each_mut<F, R, E>(&mut self, mut func : F) -> Result<[R; #fields_count], E>
                        where 
                            F : FnMut(&mut Self::Comp, usize) -> Result<R, E>,
                            R : Copy + Default 
                        {
                            let mut res = [R::default(); #fields_count];
                            #try_for_each_mut_s     // Insert stream
                            Ok(res)
                        }
                    
                        fn for_each_dyn<'a, F, R>(&'a self, mut func : F) -> Vec<R>
                        where 
                            F : FnMut(&'a Self::Comp, usize) -> R 
                        {
                            let mut res = Vec::with_capacity(#fields_count);
                            #for_each_dyn_s
                            res
                        }
                    }
                }.into()
            },
            _ => panic!("This macro can only be used on structs")
        }
    }

    #[proc_macro_derive(SyncCompGroup)]
    pub fn sync_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(input).unwrap();
        sync_comp_group_impl(ast, TokenStream::from_str("dyn syact::comp::SyncComp").unwrap())
    }
// 

// StepperCompGroup
    /// Implementation for the `StepperCompGroup` trait
    fn stepper_comp_group_impl(ast : DeriveInput) -> proc_macro::TokenStream {
        match ast.data {
            syn::Data::Struct(data) => {
                // The macro only works on structs
                let name = ast.ident;
                let fields = data.fields;
                let fields_count = fields.len();

                // Create an empty implementation
                quote::quote! {
                    impl syact::comp::stepper::StepperCompGroup<dyn syact::comp::stepper::StepperComp, #fields_count> for #name { }
                }.into()
            },
            _ => panic!("This macro can only be used on structs")
        }
    }

    #[proc_macro_derive(StepperCompGroup)]
    pub fn stepper_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
        let ast : DeriveInput = syn::parse(input).unwrap();
        let mut derive = sync_comp_group_impl(ast.clone(), TokenStream::from_str("dyn syact::comp::stepper::StepperComp").unwrap()); 
        derive.extend(stepper_comp_group_impl(ast));
        derive
    }
// 